#ifndef T1IPC_COMM_MAIN_HPP
#define T1IPC_COMM_MAIN_HPP

#include <mqueue.h>
#include <string>
#include <atomic>
#include <sys/types.h>

class T1Socket;
bool FileRecvFromAppComm(T1Socket *pmySocket, const std::string& save_path, std::string& received_filename);

#define COMM_MSG_MAX_SIZE  1024

#define MAIN_TO_COMM_QUEUE  "/theone_m2c"
#define COMM_TO_MAIN_QUEUE  "/theone_c2m"

#define MSG_SEND_DATA   'S'
#define MSG_HEARTBEAT   'H'
#define MSG_RESPONSE    'R'
#define MSG_FILE_REQ    'F'
#define MSG_QUIT        'Q'
#define MSG_COMM_READY  'Z'
#define MSG_RECV_DATA   'D'
#define MSG_NOTIFY      'N'
#define MSG_FILE_DONE   'V'

struct CommMessage {
    char type;
    char data[COMM_MSG_MAX_SIZE];
};

class T1IPCCommMain {
public:
    T1IPCCommMain();
    ~T1IPCCommMain();

    int InitMainSide();
    int InitCommSide();
    void Cleanup();

    int SendToComm(char type, const char* data);
    int RecvFromComm(char* data, int maxLen, int timeoutMs);

    int SendToMain(char type, const char* data);
    int RecvFromMain(char* data, int maxLen, int timeoutMs);

    void Stop();
    bool IsRunning() const { return running; }

private:
    mqd_t mqMainToComm;
    mqd_t mqCommToMain;
    std::atomic<bool> running;
};

#endif