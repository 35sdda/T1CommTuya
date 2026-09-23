#ifndef T1MQTT_HPP
#define T1MQTT_HPP

#include <stdio.h>
#include <mosquitto.h>
#include <string>
#include <mutex>
#include <ctime>

class T1Mqtt {
public:
    /* construct the client object, the mosquitto instance is created later in Init()
     * 2026.09.15: modified by Lff（+AI）, add function header comment
     */
    T1Mqtt();

    /* destroy the client object, disconnect from the broker and release the global MQTT library
     * 2026.09.15: modified by Lff（+AI）, add function header comment
     * 2026.09.18: modified by Lff（+AI）, the libcurl global state is left to the process exit
     */
    ~T1Mqtt();

    /* save the broker settings and create the MQTT client
     * broker = broker address, e.g. "13.50.52.37"
     * port = broker port, default 1883
     * topic = topic to subscribe, e.g. "x3m/upgrade/url"
     * username = login user, NULL for anonymous
     * password = login password, NULL for anonymous
     * return:  0, client created and the async connect started;   -1, client creation failed
     * 2026.09.15: modified by Lff（+AI）, add function header comment
     */
    int Init(const char *broker, int port, const char *topic,
             const char *username, const char *password);

    /* drive the MQTT network I/O from the main loop and keep the broker link alive, never blocks
     * return:  always 0, reconnect and the offline warning are handled inside
     * 2026.09.15: modified by Lff（+AI）, add function header comment
     */
    int Select();

    /* publish one message to the device status topic
     * data = NUL terminated payload text, published with QoS0 and retain=false
     * return:  0, published;   <0, mosquitto error code;   -1, client not ready
     * 2026.09.15: modified by Lff（+AI）, add function header comment
     */
    int SendData(const char *data);

    /* report whether Init() has been done and the client exists
     * return:  true, ready to publish;   false, not initialized
     * 2026.09.15: modified by Lff（+AI）, add function header comment
     */
    bool IsReady() const { return m_ready; }

    /* set the topic used to upload the device status
     * topic = publish topic, NULL or "" falls back to the subscribe topic
     * return:  always 0
     * 2026.09.15: modified by Lff（+AI）, add function header comment
     */
    int SetPubTopic(const char *topic);

private:
    struct mosquitto *m_mosq;       // MQTT client instance pointer
    std::string m_topic;            // subscribed topic name
    std::string m_pubTopic;         // publish topic for device status
    bool m_ready;                   // init done flag

    std::string m_broker;           // broker address
    int m_port;                     // broker port
    bool m_connected;               // connected to broker flag
    bool m_connectPending;          // a connect attempt is in flight, wait for its callback
    bool m_offlineWarned;           // offline warning already printed for this outage
    std::string m_user;             // username, kept for every reconnect
    std::string m_pass;             // password, kept for every reconnect
    time_t m_lastAlive;             // last time a live connection was seen
    time_t m_connectStart;          // start time of the last connect attempt

    std::string m_lastVer;          // last version dispatched for download
    bool m_downloading;             // download thread running flag
    std::mutex m_otaMutex;          // guards m_lastVer / m_downloading

    // OTA upgrade handling, kept inside MQTT module

    /* parse the upgrade json received on the subscribe topic and start one download thread
     * json = message body, expects the fields "url":"..." and "ver":"..."
     * 2026.09.15: modified by Lff（+AI）, add function header comment
     */
    void HandleUpgradeJson(const std::string& json);

    /* read the installed version from ../Ver/Version
     * return:  version string without the line end;   "" when the file is missing or empty
     * 2026.09.15: modified by Lff（+AI）, add function header comment
     */
    std::string ReadLocalVersion();

    /* verify a downloaded upgrade package with unzip -t
     * zip_path = package path relative to the working directory, deleted when broken
     * return:  true, package is readable;   false, package removed
     * 2026.09.15: modified by Lff（+AI）, add function header comment
     */
    bool CheckZipIntegrity(const std::string& zip_path);

    /* download the package with libcurl and verify it, then clear the version lock
     * self = T1Mqtt instance owning m_lastVer / m_downloading
     * ver = version string used in the local file name UpdateV<ver>.zip
     * url = http url taken from the upgrade json
     * 2026.09.15: modified by Lff（+AI）, add function header comment
     * 2026.09.18: modified by Lff（+AI）, download inside this process, no child process is
     *             spawned any more so nothing can outlive the1Comm
     */
    static void DownloadWorker(T1Mqtt *self, std::string ver, std::string url);

    // single threaded mode: StartClient creates the client, Select() drives it

    /* create the client, set the callbacks and start the async connect
     * return:  0, connect started or left to Select() for retry;   -1, client creation failed
     * 2026.09.15: modified by Lff（+AI）, add function header comment
     */
    int StartClient();

    // static callbacks for libmosquitto

    /* libmosquitto connect callback, marks the link online and subscribes the upgrade topic
     * rc = 0 ok; 1 protocol error; 2 client id rejected; 3 server unavailable; 4/5 auth failed
     * 2026.09.15: modified by Lff（+AI）, add function header comment
     */
    static void on_connect_cb(struct mosquitto *mosq, void *userdata, int rc);

    /* libmosquitto disconnect callback, clears the online flags so Select() retries
     * rc = 0 clean disconnect, otherwise the error code from the broker
     * 2026.09.15: modified by Lff（+AI）, add function header comment
     */
    static void on_disconnect_cb(struct mosquitto *mosq, void *userdata, int rc);

    /* libmosquitto message callback, only payloads starting with '{' are handled as upgrade json
     * msg = received message, payload / payloadlen hold the raw body
     * 2026.09.15: modified by Lff（+AI）, add function header comment
     */
    static void on_message_cb(struct mosquitto *mosq, void *userdata,
                              const struct mosquitto_message *msg);
};

#endif
