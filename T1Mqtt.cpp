#include "T1Mqtt.hpp"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <dirent.h>
#include <thread>
#include <unistd.h>
#include <sys/wait.h>
#include <curl/curl.h>

/* build a client id from the MAC of the first usable NIC, so several boards on the same
 * broker do not kick each other offline
 * return:  "x3m_<12 hex digits>" built from the NIC MAC;   "x3m_the1Comm" when no usable MAC was found
 * 2026.09.15: modified by Lff（+AI）, add function header comment
 */
static std::string MakeClientId() {
    DIR *dir=opendir("/sys/class/net");
    if ( !dir ) return "x3m_the1Comm";   // fallback: no sysfs, keep the old fixed id

    std::string mac;
    struct dirent *ent=NULL;
    while ( (ent=readdir(dir))!=NULL ) {
        if ( ent->d_name[0]=='.' ) continue;    // skip "." and ".."
        // only real NICs: lo / sit0 / tunl0 have no usable MAC
        bool wired=(strncmp(ent->d_name, "eth", 3)==0 || strncmp(ent->d_name, "en", 2)==0);
        if ( !wired && strncmp(ent->d_name, "wl", 2)!=0 ) continue;
        std::string path="/sys/class/net/";
        path+=ent->d_name;
        path+="/address";
        FILE *fp=fopen(path.c_str(), "r");
        if ( !fp ) continue;
        char buf[32]={0};
        std::string addr;
        if ( fgets(buf, sizeof(buf), fp) ) {
            addr=buf;
            while ( !addr.empty() && (addr[addr.size()-1]=='\n' || addr[addr.size()-1]=='\r') ) {
                addr.erase(addr.size()-1);
            }
            if ( addr.size()!=17 || addr=="00:00:00:00:00:00" ) addr="";   // keep only a real MAC
        }
        fclose(fp);
        if ( addr.empty() ) continue;

        mac=addr;
        if ( wired ) break;     // prefer a wired NIC, wireless only as fallback
    }
    closedir(dir);

    if ( mac.empty() ) return "x3m_the1Comm";   // fallback when no usable MAC was found

    std::string id="x3m_";
    for ( size_t i=0; i<mac.size(); i++ ) {
        if ( mac[i]!=':' ) id+=mac[i];        // strip colons: 94:a9:b8 -> 94a9b8
    }
    return id;
}
/* construct the client object, the mosquitto instance is created later in Init()
 * 2026.09.15: modified by Lff（+AI）, add function header comment
 * 2026.09.18: modified by Lff（+AI）, init the global libcurl state here, this object is
 *             built in main before any worker thread is started
 */
T1Mqtt::T1Mqtt()
    : m_mosq(NULL), m_ready(false), m_broker(""), m_port(0), m_connected(false),
      m_connectPending(false), m_offlineWarned(false), m_lastAlive(0),
      m_connectStart(0), m_lastVer(""), m_downloading(false) {
    int debug=1;
    // libcurl 7.81 predates the implicit global init added in 7.84, and curl_global_init()
    // is not thread safe: call it here, before any download thread exists
    if ( debug>1 ) printf("T1Mqtt-T1Mqtt: curl_global_init\n");
    curl_global_init(CURL_GLOBAL_DEFAULT);
}

/* destroy the client object, disconnect from the broker and release the global MQTT library
 * 2026.09.15: modified by Lff（+AI）, add function header comment
 * 2026.09.18: modified by Lff（+AI）, leave the global libcurl state to the process exit
 */
T1Mqtt::~T1Mqtt() {
    if ( m_mosq ) {
        mosquitto_disconnect(m_mosq);   // disconnect from broker
        mosquitto_destroy(m_mosq);      // free client instance
    }
    mosquitto_lib_cleanup();            // cleanup global library
    // curl_global_cleanup() is deliberately NOT called here: the download thread is detached
    // and may still be inside curl_easy_perform() while this destructor runs, and libcurl
    // requires the global cleanup to happen after all libcurl usage has ended. The process
    // is exiting anyway, the kernel reclaims the libcurl state together with the memory.
}

/* save the broker settings and create the MQTT client
 * broker = broker address, e.g. "13.50.52.37", NULL keeps an empty address
 * port = broker port, 1883 for plain TCP
 * topic = topic to subscribe, e.g. "x3m/upgrade/url"
 * username = login user, NULL means anonymous
 * password = login password, NULL means anonymous
 * return:  0, client created and the async connect started;   -1, client creation failed
 * 2026.09.15: modified by Lff（+AI）, add function header comment
 */
int T1Mqtt::Init(const char *broker, int port, const char *topic,
                 const char *username, const char *password) {
    m_topic=topic?topic:"";
    m_pubTopic=m_topic;                 // default: same as subscribe topic
    m_broker=broker?broker:"";
    m_port=port;
    m_user=username?username:"";        // kept for every reconnect
    m_pass=password?password:"";
    m_lastAlive=time(NULL);             // offline watchdog start point

    mosquitto_lib_init();               // init global MQTT library (once)
    return StartClient();
}

/* create the mosquitto client, register the callbacks and start the async connect
 * return:  0, connect started or left to Select() for retry;   -1, client creation failed
 * 2026.09.15: modified by Lff（+AI）, add function header comment
 */
int T1Mqtt::StartClient() {
    int debug=1;

    // mosquitto_new: create client instance
    // param1: client id, should be unique per device
    // param2: clean_session, true=server discards offline messages on disconnect
    // param3: userdata, passed to all callbacks as this pointer
    std::string clientId=MakeClientId();   // unique per board
    m_mosq=mosquitto_new(clientId.c_str(), true, this);
    if ( !m_mosq ) {
        if ( debug>0 ) printf("T1Mqtt-StartClient: failed to create mosquitto instance\n");
        return -1;
    }

    // set connection success callback
    mosquitto_connect_callback_set(m_mosq, on_connect_cb);
    // set message receive callback
    mosquitto_message_callback_set(m_mosq, on_message_cb);
    // set disconnect callback
    mosquitto_disconnect_callback_set(m_mosq, on_disconnect_cb);

    // set auth if username/password provided
    if ( !m_user.empty() && !m_pass.empty() ) {
        mosquitto_username_pw_set(m_mosq, m_user.c_str(), m_pass.c_str());
    }

    // mosquitto_connect_async: non-blocking connect, finished by mosquitto_loop()
    // param1: client instance
    // param2: broker address
    // param3: port number
    // param4: keepalive interval (seconds), 60s ping interval
    m_connectStart=time(NULL);
    int ret=mosquitto_connect_async(m_mosq, m_broker.c_str(), m_port, 60);
    if ( ret==MOSQ_ERR_SUCCESS ) {
        m_connectPending=true;          // callbacks clear it when the attempt is over
    } else {
        // do not bail out: the broker may be unresolvable at boot,
        // Select() keeps retrying every RECONNECT_INTERVAL_SEC
        if ( debug>0 ) printf("T1Mqtt-StartClient: connect_async failed: %s, will retry\n", mosquitto_strerror(ret));
        fflush(stdout);
    }

    m_ready=true;
    if ( debug>0 ) printf("T1Mqtt-StartClient: connecting to %s:%d, topic=%s, client=%s\n",
                          m_broker.c_str(), m_port, m_topic.c_str(), clientId.c_str());
    return 0;
}

// packets handled per main loop round, keeps a single round short
static const int MQTT_LOOP_MAX_PACKETS=4;
// retry the broker every this many seconds while offline
static const int RECONNECT_INTERVAL_SEC=2;
// abandon a connect attempt that produced no callback within this many seconds
static const int CONNECT_TIMEOUT_SEC=30;
// log one warning when the link stays down this long
static const int OFFLINE_WARN_SEC=120;

/* drive the MQTT network I/O from the main loop and keep the broker link alive, never blocks
 * return:  always 0, reconnect and the offline warning are handled inside
 * 2026.09.15: modified by Lff（+AI）, add function header comment
 */
int T1Mqtt::Select() {
    int debug=1;
    if ( !m_ready || !m_mosq ) return 0;

    // single threaded mode: drive the network I/O from the main loop.
    // timeout=0 means poll only, so the IPC loop is never blocked here
    int ret=mosquitto_loop(m_mosq, 0, MQTT_LOOP_MAX_PACKETS);
    if ( ret!=MOSQ_ERR_SUCCESS && ret!=MOSQ_ERR_NO_CONN ) {
        if ( debug>1 ) printf("T1Mqtt-Select: loop rc=%s\n", mosquitto_strerror(ret));
    }

    time_t now=time(NULL);
    if ( m_connected ) {
        m_lastAlive=now;                // online, refresh the offline watchdog
        m_offlineWarned=false;
        return 0;
    }

    // still connecting: wait for the callback, unless the attempt went silent
    if ( m_connectPending && now-m_connectStart<CONNECT_TIMEOUT_SEC ) return 0;

    if ( !m_offlineWarned && now-m_lastAlive>=OFFLINE_WARN_SEC ) {
        m_offlineWarned=true;           // warn once per outage, retries continue
        if ( debug>0 ) printf("T1Mqtt-Select: offline for %ds, still retrying\n", (int)(now-m_lastAlive));
        fflush(stdout);
    }

    if ( now-m_connectStart<RECONNECT_INTERVAL_SEC ) return 0;

    m_connectStart=now;
    m_connectPending=true;
    ret=mosquitto_reconnect_async(m_mosq);  // non blocking: only starts the TCP connect
    if ( ret==MOSQ_ERR_SUCCESS ) {
        if ( debug>1 ) printf("T1Mqtt-Select: reconnect_async started, offline for %ds\n", (int)(now-m_lastAlive));
    } else {
        m_connectPending=false;         // not started at all, allow the next retry
        if ( debug>0 ) printf("T1Mqtt-Select: reconnect_async failed: %s\n", mosquitto_strerror(ret));
        fflush(stdout);
    }
    return 0;
}

/* set the topic used to upload the device status
 * topic = publish topic, NULL or "" falls back to the subscribe topic
 * return:  always 0
 * 2026.09.15: modified by Lff（+AI）, add function header comment
 */
int T1Mqtt::SetPubTopic(const char *topic) {
    // NULL or empty topic falls back to the subscribe topic
    m_pubTopic=(topic && topic[0]!='\0')?topic:m_topic;
    return 0;
}

/* publish one message, used to forward the status text produced by the1 / the1Act
 * data = NUL terminated payload text, published with QoS0 and retain=false
 * return:  0, published;   <0, mosquitto error code;   -1, client not ready
 * 2026.09.15: modified by Lff（+AI）, add function header comment
 */
int T1Mqtt::SendData(const char *data) {
    if ( !m_ready || !m_mosq ) return -1;

    // mosquitto_publish: publish message to topic
    // param1: client instance
    // param2: message_id pointer (can be NULL)
    // param3: topic string
    // param4: message length
    // param5: message content
    // param6: QoS level (0=at most once, 1=at least once, 2=exactly once)
    // param7: retain flag (true=broker keeps last message for new subscribers)
    int ret=mosquitto_publish(m_mosq, NULL, m_pubTopic.c_str(),
                                strlen(data), data, 0, false);
    return ret;
}

/* parse the upgrade json received on the subscribe topic and start one download thread
 * json = message body, expects the fields "url":"..." and "ver":"..."
 * 2026.09.15: modified by Lff（+AI）, add function header comment
 */
/* compare two version numbers parsed as x.y.z
 * v1, v2 = version arrays of length 3
 * return:  1 if v1>v2;   -1 if v1<v2;   0 if equal
 * 2026.09.17: created by Lff（+AI）
 */
static int cmpVer(int v1[3], int v2[3]) {
    for ( int i=0; i<3; i++ ) {
        if ( v1[i]!=v2[i] ) return v1[i]>v2[i] ? 1 : -1;
    }
    return 0;
}

void T1Mqtt::HandleUpgradeJson(const std::string& json) {
    int debug=1;
    char url[512]={0};
    char ver[64]={0};
    const char *p=strstr(json.c_str(), "\"url\":\"");
    if ( p ) sscanf(p+7, "%511[^\"]", url);    // skip "url":, cap 511 to fit url[512]
    p=strstr(json.c_str(), "\"ver\":\"");
    if ( p ) sscanf(p+7, "%63[^\"]", ver);    // skip "ver":, cap 63 to fit ver[64]

    if ( url[0]=='\0' || ver[0]=='\0' ) return;

    std::lock_guard<std::mutex> lock(m_otaMutex);
    // dedup: retained msg redelivery, download busy, or already installed
    if ( m_downloading || m_lastVer==ver || ReadLocalVersion()==ver ) {
        if ( debug>0 ) printf("T1Mqtt-HandleUpgradeJson: skip ver=%s\n", ver);
        fflush(stdout);
        return;
    }

    // skip anything that is not strictly newer than the running version: the broker keeps
    // retransmitting the retained upgrade message, an older package would be fetched again
    // on every reconnect (the server apk folder may lag behind the board)
    int newVer[3]={0};
    int curVer[3]={0};
    std::string curVerStr=ReadLocalVersion();
    if ( sscanf(ver, "%d.%d.%d", &newVer[0], &newVer[1], &newVer[2])==3 &&
         sscanf(curVerStr.c_str(), "%d.%d.%d", &curVer[0], &curVer[1], &curVer[2])==3 &&
         cmpVer(newVer, curVer)<=0 ) {
        if ( debug>0 ) printf("T1Mqtt-HandleUpgradeJson: skip ver=%s, not newer than running %s\n",
                              ver, curVerStr.c_str());
        fflush(stdout);
        return;
    }

    m_lastVer=ver;
    m_downloading=true;
    if ( debug>0 ) printf("T1Mqtt-HandleUpgradeJson: download ver=%s started in thread\n", ver);
    std::thread t(DownloadWorker, this, std::string(ver), std::string(url));
    t.detach();
}

/* read the installed version from ../Ver/Version
 * return:  version string without the line end;   "" when the file is missing or empty
 * 2026.09.15: modified by Lff（+AI）, add function header comment
 */
std::string T1Mqtt::ReadLocalVersion() {
    FILE *fp=fopen("../Ver/Version", "r");
    char buf[64]={0};
    std::string ver;
    if ( fp ) {
        if ( fgets(buf, sizeof(buf), fp) ) ver=buf;
        fclose(fp);
    }
    while ( !ver.empty() && (ver[ver.size()-1]=='\n' || ver[ver.size()-1]=='\r') ) {
        ver.erase(ver.size()-1);
    }
    return ver;
}

/* verify a downloaded upgrade package with unzip -t
 * zip_path = package path relative to the working directory, deleted when broken
 * return:  true, package is readable;   false, package removed
 * 2026.09.15: modified by Lff（+AI）, add function header comment
 */
bool T1Mqtt::CheckZipIntegrity(const std::string& zip_path) {
    int debug=1;
    char cmd[512]={0};
    snprintf(cmd, sizeof(cmd), "unzip -t \"%s\"", zip_path.c_str());
    int ret=system(cmd);
    if ( WIFEXITED(ret) && WEXITSTATUS(ret)==0 ) {
        if ( debug>1 ) printf("T1Mqtt-CheckZipIntegrity: Zip integrity OK: %s\n", zip_path.c_str());
        return true;
    }
    if ( debug>0 ) printf("T1Mqtt-CheckZipIntegrity: Zip integrity FAILED, deleting: %s\n", zip_path.c_str());
    remove(zip_path.c_str());
    return false;
}

/* download the package with libcurl and verify it, then clear the version lock
 * self = T1Mqtt instance owning m_lastVer / m_downloading
 * ver = version string used in the local file name UpdateV<ver>.zip
 * url = http url taken from the upgrade json
 * 2026.09.15: modified by Lff（+AI）, add function header comment
 * 2026.09.17: modified by Lff（+AI）, spawn wget by fork/exec instead of system() so it
 *             inherits no fd (port 1111 stays free), and bind it to the1Comm lifetime
 *             with PR_SET_PDEATHSIG so wget dies together with the1Comm
 * 2026.09.18: modified by Lff（+AI）, download with libcurl inside this process: the worker
 *             thread dies together with the1Comm, so no extra process and no inherited fd
 *             can outlive the1Comm and hold the LISTEN socket of port 1111
 */
void T1Mqtt::DownloadWorker(T1Mqtt *self, std::string ver, std::string url) {
    // run download + integrity check in a dedicated thread, no event loop blocking
    int debug=1;
    char zipPath[256]={0};
    char zipPart[256]={0};
    snprintf(zipPath, sizeof(zipPath), "../Ver/UpdateV%s.zip", ver.c_str());
    snprintf(zipPart, sizeof(zipPart), "%s.part", zipPath);   // download to *.part first
    if ( debug>0 ) printf("T1Mqtt-DownloadWorker: downloading ver=%s ...\n", ver.c_str());
    fflush(stdout);

    // download inside this process: no child is spawned, so nothing can outlive the1Comm and
    // keep the LISTEN socket of port 1111 open (T1Socket sets no FD_CLOEXEC)
    int ret=-1;
    CURL *curl=curl_easy_init();
    if ( curl ) {
        FILE *fp=fopen(zipPart, "wb");
        if ( fp ) {
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
            curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);   // follow 3xx redirects
            curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);      // http >=400 becomes an error
            curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);  // seconds, connect phase only
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 1800L);       // seconds, whole transfer cap
            curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);         // required when using threads
            CURLcode rc=curl_easy_perform(curl);
            fclose(fp);
            if ( rc==CURLE_OK ) {
                ret=0;
            } else {
                if ( debug>0 ) printf("T1Mqtt-DownloadWorker: download failed, curl rc=%d (%s)\n",
                                      rc, curl_easy_strerror(rc));
            }
        } else {
            if ( debug>0 ) printf("T1Mqtt-DownloadWorker: cannot open %s\n", zipPart);
        }
        curl_easy_cleanup(curl);
    } else {
        if ( debug>0 ) printf("T1Mqtt-DownloadWorker: curl_easy_init failed\n");
    }

    // publish the package only when the transfer finished cleanly: rename *.part to the real
    // name, an interrupted download leaves *.part behind which the1 ignores
    if ( ret==0 && rename(zipPart, zipPath)==0 ) {
        std::string zip_path=zipPath;
        if ( !self->CheckZipIntegrity(zip_path) ) {
            std::lock_guard<std::mutex> lock(self->m_otaMutex);
            self->m_lastVer.clear();    // bad zip already deleted, clear version to allow retry
        }
    } else {
        if ( debug>0 ) printf("T1Mqtt-DownloadWorker: no package published, ret=%d\n", ret);
        remove(zipPart);                // drop the incomplete download
        std::lock_guard<std::mutex> lock(self->m_otaMutex);
        self->m_lastVer.clear();        // clear version, allow retry
    }

    std::lock_guard<std::mutex> lock(self->m_otaMutex);
    self->m_downloading=false;
    fflush(stdout);
}

/* libmosquitto connect callback, marks the link online and subscribes the upgrade topic
 * mosq = client instance, unused
 * userdata = T1Mqtt instance, restored to the this pointer
 * rc = 0 ok; 1 protocol error; 2 client id rejected; 3 server unavailable; 4/5 auth failed
 * 2026.09.15: modified by Lff（+AI）, add function header comment
 */
void T1Mqtt::on_connect_cb(struct mosquitto *mosq, void *userdata, int rc) {
    int debug=1;
    T1Mqtt *self=(T1Mqtt *)userdata;  // restore object pointer from userdata

    if ( rc==0 ) {
        self->m_connected=true;
        self->m_connectPending=false;
        self->m_offlineWarned=false;
        self->m_lastAlive=time(NULL);   // refresh the offline watchdog
        if ( debug>0 ) printf("T1Mqtt-on_connect_cb: connected to broker\n");

        // mosquitto_subscribe: subscribe to topic
        // param1: client instance
        // param2: message_id pointer (can be NULL)
        // param3: topic string (supports wildcards + and #)
        // param4: QoS level
        mosquitto_subscribe(mosq, NULL, self->m_topic.c_str(), 0);
        if ( debug>0 ) printf("T1Mqtt-on_connect_cb: subscribed to %s\n", self->m_topic.c_str());
    } else {
        self->m_connected=false;
        self->m_connectPending=false;   // refused, Select() retries later
        if ( debug>0 ) printf("T1Mqtt-on_connect_cb: connect failed, rc=%d\n", rc);
        // rc=1 protocol error, 2 id rejected, 3 server unavailable, 4/5 auth fail
    }
}

/* libmosquitto disconnect callback, clears the online flags so Select() retries
 * mosq = client instance, unused
 * userdata = T1Mqtt instance, restored to the this pointer
 * rc = 0 clean disconnect, otherwise the error code from the broker
 * 2026.09.15: modified by Lff（+AI）, add function header comment
 */
void T1Mqtt::on_disconnect_cb(struct mosquitto *mosq, void *userdata, int rc) {
    int debug=1;
    T1Mqtt *self=(T1Mqtt *)userdata;

    self->m_connected=false;
    self->m_connectPending=false;       // Select() starts the next attempt
    if ( debug>0 ) printf("T1Mqtt-on_disconnect_cb: disconnected, rc=%d, will retry\n", rc);
    fflush(stdout);
}

/* libmosquitto message callback, only payloads starting with '{' are handled as upgrade json
 * mosq = client instance, unused
 * userdata = T1Mqtt instance, restored to the this pointer
 * msg = received message, payload / payloadlen hold the raw body
 * 2026.09.15: modified by Lff（+AI）, add function header comment
 */
void T1Mqtt::on_message_cb(struct mosquitto *mosq, void *userdata,
                            const struct mosquitto_message *msg) {
    int debug=1;
    T1Mqtt *self=(T1Mqtt *)userdata;

    if ( !msg || !msg->payload || msg->payloadlen<=0 ) return;

    // mosquitto_message members:
    //   mid:      message id
    //   topic:    topic this message arrived on
    //   payload:  message data (void* pointer)
    //   payloadlen: message length in bytes
    //   qos:      QoS level of this message
    //   retain:   whether this is a retained message

    std::string payload((const char *)msg->payload, (size_t)msg->payloadlen);

    // only print none status message
    if ( (int)payload.size()>100 ) {
        if ( debug>1 ) printf("T1Mqtt-on_message_cb: msg on %s, len=%d\n", msg->topic, (int)payload.size());
    }

    // ignore status messages echoed back, only process upgrade json
    if ( !payload.empty() && payload[0]=='{' ) {
        if ( debug>0 ) printf("T1Mqtt-on_message_cb: got msg: %s\n", payload.c_str());
        self->HandleUpgradeJson(payload);
    }
}
