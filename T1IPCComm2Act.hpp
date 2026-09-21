#ifndef T1IPC_COMM2ACT_HPP
#define T1IPC_COMM2ACT_HPP

#include <mqueue.h>
#include <string>

#define ACTCOMM_MSG_MAX_SIZE  1024

#define ACT2COMM_QUEUE  "/theone_a2c"
#define COMM2ACT_QUEUE  "/theone_c2a"

#define ACTCOMM_CMD_MAP_SWITCH   'M'
#define ACTCOMM_CMD_MAP_DELETE   'D'
#define ACTCOMM_CMD_MAP_RECV     'R'
#define ACTCOMM_CMD_MAP_LIST     'L'
#define ACTCOMM_CMD_QUIT         'X'
#define ACTCOMM_CMD_PING         'P'
#define ACTCOMM_CMD_APP          'A'

#define ACTCOMM_STATUS           'S'
#define ACTCOMM_MAP_LIST         'l'
#define ACTCOMM_MAP_CONTENT      'm'
#define ACTCOMM_HEARTBEAT        'H'
#define ACTCOMM_NOTIFY           'N'
#define ACTCOMM_READY            'Y'

struct ActCommMessage {
    char type;
    char data[ACTCOMM_MSG_MAX_SIZE];
};

class T1IPCComm2Act {
public:
    T1IPCComm2Act();
    ~T1IPCComm2Act();

    int InitCommSide();
    void Cleanup();

    int SendToAct(char type, const char* data, int len = 0);
    int RecvFromAct(char* data, int maxLen, int timeoutMs);

    bool IsReady() const { return mqA2C != (mqd_t)-1 && mqC2A != (mqd_t)-1; }
    mqd_t GetA2CFd() const { return mqA2C; }
    mqd_t GetC2AFd() const { return mqC2A; }

private:
    mqd_t mqA2C;
    mqd_t mqC2A;
};

#endif