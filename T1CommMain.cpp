#include "T1IPCCommMain.hpp"
#include "T1IPCComm2Act.hpp"
#include "T1Socket.hpp"
#include "T1BlueT.hpp"
#include "T1Mqtt.hpp"
#include "T1Tuya.hpp"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <string>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <ctime>

static T1IPCCommMain *g_comm = nullptr;
static T1BlueT *g_blueT = nullptr;
static T1Socket *g_socket = nullptr;
static T1Mqtt *g_mqtt = nullptr;
static T1Tuya *g_tuya = nullptr;
static volatile bool g_shutdown = false;

static void signalHandler(int sig) {
    int debug=0;
    if (debug>0) printf("signalHandler: Signal %d received, shutting down...\n", sig);
    g_shutdown = true;
    if ( g_comm ) g_comm->Stop();
}

static int SetSysTime(const char* buf, int buf_len) {
    int debug=0;
    if (buf == nullptr || buf_len < 17 || buf[0] != '$' || buf[1] != 'T' || buf[2] != ':') {
        if (debug>0) printf("SetSysTime: invalid format, len=%d\n", buf_len);
        return -1;
    }

    int year = (buf[3]-'0')*1000 + (buf[4]-'0')*100 + (buf[5]-'0')*10 + (buf[6]-'0');
    int month = (buf[7]-'0')*10 + (buf[8]-'0');
    int day = (buf[9]-'0')*10 + (buf[10]-'0');
    int hour = (buf[11]-'0')*10 + (buf[12]-'0');
    int min = (buf[13]-'0')*10 + (buf[14]-'0');
    int sec = (buf[15]-'0')*10 + (buf[16]-'0');

    if (year < 2000 || year > 2100 || month < 1 || month > 12 || day < 1 || day > 31) {
        if (debug>0) printf("SetSysTime: invalid time values\n");
        return -2;
    }

    char cmd[100];
    snprintf(cmd, sizeof(cmd), "sudo date -s '%d-%02d-%02d %02d:%02d:%02d'", year, month, day, hour, min, sec);
    int ret = system(cmd);

    if (ret != 0) {
        if (debug>0) printf("SetSysTime: failed, ret=%d\n", ret);
        return -3;
    }

    if (debug>1) printf("SetSysTime: ok: %d-%02d-%02d %02d:%02d:%02d\n", year, month, day, hour, min, sec);
    return 0;
}

static bool IsAppMsg(const char *buf, int len) {
    return len > 3 && buf[0] == '$' && buf[2] == ':';
}

static bool CheckZipIntegrity(const std::string& zip_path) {
    int debug=0;
    char cmd[512] = {0};
    snprintf(cmd, sizeof(cmd), "unzip -t \"%s\"", zip_path.c_str());
    int ret = system(cmd);
    if (WIFEXITED(ret) && WEXITSTATUS(ret) == 0) {
        if (debug>1) printf("CheckZipIntegrity: OK: %s\n", zip_path.c_str());
        return true;
    } else {
        if (debug>0) printf("CheckZipIntegrity: FAILED, deleting: %s\n", zip_path.c_str());
        remove(zip_path.c_str());
        return false;
    }
}


static void CommEventLoop(T1IPCCommMain& comm, T1BlueT& blueT, T1Socket& socket, T1Mqtt& mqtt, T1Tuya& tuya, T1IPCComm2Act* comm2act) {
    int debug=0;
    bool running = true;

    while ( running ) {
        bool hasOutput = false;
        int blueRet = 0;
        int sockRet = 0;

        char outBuf[COMM_MSG_MAX_SIZE + 1];
        int msgType = comm.RecvFromMain(outBuf, COMM_MSG_MAX_SIZE, 0);
        while ( msgType>0 ) {
            switch ( msgType ) {
            case MSG_SEND_DATA:
            case MSG_HEARTBEAT:
            case MSG_RESPONSE:
                if ( blueT.meclientsock>0 ) {
                    blueT.SendData(outBuf);
                }
                socket.SendData(outBuf);
                if (mqtt.IsReady()) mqtt.SendData(outBuf);
                if (tuya.IsReady()) tuya.SendData(outBuf);
                hasOutput = true;
                break;
            case MSG_FILE_REQ:
                blueT.SendOneMapFile(outBuf);
                socket.SendOneMapFile(outBuf);
                hasOutput = true;
                break;
            case MSG_QUIT:
                running = false;
                break;
            }
            msgType = comm.RecvFromMain(outBuf, COMM_MSG_MAX_SIZE, 0);
        }

        if (comm2act && comm2act->IsReady()) {
            char actBuf[ACTCOMM_MSG_MAX_SIZE + 1];
            int atype = comm2act->RecvFromAct(actBuf, sizeof(actBuf), 0);
            while (atype > 0) {
                switch (atype) {
                case ACTCOMM_STATUS:
                case ACTCOMM_HEARTBEAT:
                case ACTCOMM_MAP_LIST:
                case ACTCOMM_MAP_CONTENT:
                case ACTCOMM_NOTIFY:
                    if (blueT.meclientsock > 0) {
                        blueT.SendData(actBuf);
                    }
                    socket.SendData(actBuf);
                    if (mqtt.IsReady()) mqtt.SendData(actBuf);
                    if (tuya.IsReady()) tuya.SendData(actBuf);
                    hasOutput = true;
                    break;
                default:
                    if (debug>1) printf("CommEventLoop: A2C Unknown type=%d\n", atype);
                    break;
                }
                atype = comm2act->RecvFromAct(actBuf, sizeof(actBuf), 0);
            }
        }

        if ( blueT.meserversock>0 ) {
            blueRet = blueT.Select();
            if ( blueRet>0 ) {
                for ( int i=0; i<=blueT.memaxfd; i++ ) {
                    if ( !FD_ISSET(i, &(blueT.mereadfds)) ) continue;

                    if ( i==blueT.meserversock ) {
                        if ( blueT.AcceptClient()<0 ) {
                            if (debug>0) printf("CommEventLoop: Failed to accept bluetooth client.\n");
                            break;
                        }
                        if (debug>1) printf("CommEventLoop: Bluetooth client connected.\n");
                        CommMessage notif;
                        notif.type = MSG_NOTIFY;
                        strcpy(notif.data, "Bluetooth client connected");
                        comm.SendToMain(MSG_NOTIFY, notif.data);
                    } else {
                        int recv_size = blueT.RecvData(i);
                        if ( IsAppMsg(blueT.mebufRecv, recv_size) ) {
                            if (blueT.mebufRecv[1] == 'T') {
                                SetSysTime(blueT.mebufRecv, recv_size);
                            } else if (comm2act && comm2act->IsReady()) {
                                comm2act->SendToAct(ACTCOMM_CMD_APP, blueT.mebufRecv);
                            }
                        }
                    }
                }
            }
        }

        sockRet = socket.Select();
        if ( sockRet>0 ) {
            for ( int i=0; i<=socket.maxfd; i++ ) {
                if ( !FD_ISSET(i, &(socket.readfds)) ) continue;

                if ( i==socket.GetSocket('s') ) {
                    if ( socket.AcceptClient()<0 ) {
                        if (debug>0) printf("CommEventLoop: Failed to accept socket client.\n");
                        continue;
                    }
                    if (debug>1) printf("CommEventLoop: Socket client connected.\n");
                    CommMessage notif;
                    notif.type = MSG_NOTIFY;
                    strcpy(notif.data, "Socket client connected");
                    comm.SendToMain(MSG_NOTIFY, notif.data);
                } else {
                    int recv_size = socket.RecvData(i);
                    if ( IsAppMsg(socket.bufRecv, recv_size) ) {
                        if (socket.bufRecv[1] == 'T') {
                            SetSysTime(socket.bufRecv, recv_size);
                        } else if (socket.bufRecv[1] == 'F') {
                            std::string received_filename;
                            if (FileRecvFromAppComm(&socket, "../Ver/", received_filename)) {
                                std::string full_path = "../Ver/";
                                full_path += received_filename;
                                CheckZipIntegrity(full_path);
                            }
                        } else if (comm2act && comm2act->IsReady()) {
                            comm2act->SendToAct(ACTCOMM_CMD_APP, socket.bufRecv);
                        }
                    }
                }
            }
        }

        if ( mqtt.IsReady() ) {
            mqtt.Select();      // recv/parse/dedup/download all handled inside T1Mqtt
        }
        if (tuya.IsReady()) {
            tuya.Select();
            tuya.RecvData();
        }
        if ( !running ) break;
        if ( g_shutdown ) {
            if (debug>0) printf("CommEventLoop: Shutdown signal received, exiting loop.\n");
            break;
        }
        if ( getppid() == 1 ) {
            if (debug>0) printf("CommEventLoop: Parent process exited, shutting down.\n");
            break;
        }

        if ( !hasOutput && blueT.meserversock>0 && blueRet==0 && sockRet==0 ) {
            usleep(5000);
        }
        if ( !hasOutput && blueT.meserversock<=0 && sockRet==0 ) {
            usleep(5000);
        }
    }

    if (debug>0) printf("CommEventLoop: exiting.\n");
}

int main(int argc, char *argv[]) {
    int debug=0;

    setvbuf(stdout, NULL, _IOLBF, 0);   // line buffered: flush log per line even when piped

    prctl(PR_SET_PDEATHSIG, SIGTERM);

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    signal(SIGPIPE, SIG_IGN);

    T1IPCCommMain comm;
    g_comm = &comm;

    if ( comm.InitCommSide()!=0 ) {
        if (debug>0) printf("main: Failed to open IPC queues. Is the main process running?\n");
        return 1;
    }

    T1IPCComm2Act comm2act;
    bool actCommOk = (comm2act.InitCommSide() == 0);
    if (debug>1) printf("main: Comm2Act IPC %s.\n", actCommOk ? "ready" : "not available");

    T1BlueT blueT;
    T1Socket socket;
    T1Mqtt mqtt;
    T1Tuya tuya;
    g_blueT = &blueT;
    g_socket = &socket;
    g_mqtt = &mqtt;
    g_tuya = &tuya;

    int blueRet = blueT.GetSocket('s');
    bool blueOk = (blueRet==0);
    if (debug>1) {
        if (blueOk) {
            printf("main: Bluetooth SPP Server initialized\n");
        } else {
            printf("main: Bluetooth init failed, continuing without bluetooth.\n");
        }
    }

    mqtt.Init("13.50.52.37", 1883, "x3m/upgrade/url", "x3m", "robot@2026@Leo");   // broker now requires user/pass
    mqtt.SetPubTopic("x3m/device/status");   // device status no longer shares the upgrade topic
    tuya.Init();

    if (debug>1) printf("main: Socket server on port %d\n", DEFAULT_PORT);

    if (debug>0) {
        printf("main: Started OK (bluetooth=%s, actComm=%s)\n",
               blueOk ? "OK" : "skip", actCommOk ? "OK" : "skip");
    }

    comm.SendToMain(MSG_COMM_READY, nullptr);

    CommEventLoop(comm, blueT, socket, mqtt, tuya, &comm2act);

    comm.Cleanup();
    if (debug>0) printf("main: Stopped.\n");
    return 0;
}
