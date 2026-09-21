#include "T1IPCComm2Act.hpp"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <errno.h>

T1IPCComm2Act::T1IPCComm2Act() : mqA2C((mqd_t)-1), mqC2A((mqd_t)-1) {
}

T1IPCComm2Act::~T1IPCComm2Act() {
    Cleanup();
}

int T1IPCComm2Act::InitCommSide() {
    int debug=0;
    struct mq_attr attr;
    attr.mq_flags = O_NONBLOCK;
    attr.mq_maxmsg = 10;
    attr.mq_msgsize = sizeof(ActCommMessage);
    attr.mq_curmsgs = 0;

    mqA2C = mq_open(ACT2COMM_QUEUE, O_RDONLY | O_NONBLOCK);
    if (mqA2C == (mqd_t)-1) {
        if (debug>0) printf("T1IPCComm2Act-InitCommSide: failed to open %s (recv): %s\n", ACT2COMM_QUEUE, strerror(errno));
        return -1;
    }

    mqC2A = mq_open(COMM2ACT_QUEUE, O_WRONLY | O_NONBLOCK);
    if (mqC2A == (mqd_t)-1) {
        if (debug>0) printf("T1IPCComm2Act-InitCommSide: failed to open %s (send): %s\n", COMM2ACT_QUEUE, strerror(errno));
        mq_close(mqA2C);
        mqA2C = (mqd_t)-1;
        return -1;
    }

    if (debug>0) printf("T1IPCComm2Act-InitCommSide: connected %s(recv) %s(send).\n", ACT2COMM_QUEUE, COMM2ACT_QUEUE);
    return 0;
}

void T1IPCComm2Act::Cleanup() {
    if (mqA2C != (mqd_t)-1) {
        mq_close(mqA2C);
        mqA2C = (mqd_t)-1;
    }
    if (mqC2A != (mqd_t)-1) {
        mq_close(mqC2A);
        mqC2A = (mqd_t)-1;
    }
}

int T1IPCComm2Act::SendToAct(char type, const char* data, int len) {
    int debug=0;
    if (mqC2A == (mqd_t)-1) return -1;

    ActCommMessage msg;
    msg.type = type;
    if (data) {
        int copyLen = (len > 0 && len < ACTCOMM_MSG_MAX_SIZE) ? len : ACTCOMM_MSG_MAX_SIZE - 1;
        strncpy(msg.data, data, copyLen);
        msg.data[copyLen] = '\0';
    } else {
        msg.data[0] = '\0';
    }

    int ret = mq_send(mqC2A, (const char*)&msg, sizeof(msg), 0);
    if (ret == -1) {
        if (errno == EAGAIN) {
            if (debug>0) printf("T1IPCComm2Act-SendToAct: send queue full, dropped type=%c\n", type);
        } else {
            if (debug>0) printf("T1IPCComm2Act-SendToAct: mq_send failed type=%c: %s\n", type, strerror(errno));
        }
        return -1;
    }
    return 0;
}

int T1IPCComm2Act::RecvFromAct(char* data, int maxLen, int timeoutMs) {
    int debug=0;
    if (mqA2C == (mqd_t)-1) return -1;

    ActCommMessage msg;
    int ret;

    if (timeoutMs > 0) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += timeoutMs / 1000;
        ts.tv_nsec += (timeoutMs % 1000) * 1000000L;
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000L;
        }
        ret = mq_timedreceive(mqA2C, (char*)&msg, sizeof(msg), NULL, &ts);
        if (ret == -1 && errno == ETIMEDOUT) return 0;
    } else {
        ret = mq_receive(mqA2C, (char*)&msg, sizeof(msg), NULL);
        if (ret == -1 && errno == EAGAIN) return 0;
    }

    if (ret == -1) {
        if (debug>0) printf("T1IPCComm2Act-RecvFromAct: mq_receive failed: %s\n", strerror(errno));
        return -1;
    }

    if (data) {
        strncpy(data, msg.data, maxLen - 1);
        data[maxLen - 1] = '\0';
    }
    return msg.type;
}