#include "T1IPCCommMain.hpp"
#include "T1Socket.hpp"
#include "T1BlueT.hpp"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <fstream>
#include <string>
#include <arpa/inet.h>

static uint64_t ntohll(uint64_t net_val);

T1IPCCommMain::T1IPCCommMain() : mqMainToComm((mqd_t)-1), mqCommToMain((mqd_t)-1), running(false) {
}

T1IPCCommMain::~T1IPCCommMain() {
    Cleanup();
}

static uint64_t ntohll(uint64_t net_val) {
    static const int endian=1;
    if ( *(const char*)&endian==1 ) {
        uint32_t high = ntohl((uint32_t)(net_val>>32));
        uint32_t low = ntohl((uint32_t)(net_val&0xFFFFFFFF));
        return ((uint64_t)low<<32) | high;
    } else {
        return net_val;
    }
}

bool FileRecvFromAppComm(T1Socket *pmySocket, const std::string& save_path, std::string& received_filename) {
    printf("T1IPCCommMain-FileRecv: Start receiving file...\n");
    int client_fd = pmySocket->GetSocket('c');
    if ( client_fd<=0 ) {
        printf("T1IPCCommMain-FileRecv: Invalid client socket\n");
        return false;
    }

    uint32_t filename_len=0;
    int bytes_received=0;
    char len_buf[4]={0};
    while ( bytes_received<4 ) {
        int recv_size = pmySocket->RecvData(client_fd, 1);
        if ( recv_size==-1 ) {
            printf("T1IPCCommMain-FileRecv: Failed to receive filename length\n");
            return false;
        } else if ( recv_size==0 ) {
            usleep(1000);
            continue;
        }
        int copy_len = (bytes_received+recv_size)>4 ? (4-bytes_received) : recv_size;
        memcpy(len_buf+bytes_received, pmySocket->bufRecv, copy_len);
        bytes_received += copy_len;
    }
    memcpy(&filename_len, len_buf, 4);
    filename_len = ntohl(filename_len);
    printf("T1IPCCommMain-FileRecv: Filename length: %u\n", filename_len);

    if ( filename_len==0 || filename_len>1024 ) {
        printf("T1IPCCommMain-FileRecv: Invalid filename length %u\n", filename_len);
        return false;
    }

    std::string filename;
    filename.reserve(filename_len);
    bytes_received=0;
    while ( bytes_received<filename_len ) {
        int recv_size = pmySocket->RecvData(client_fd, 1);
        if ( recv_size==-1 ) {
            printf("T1IPCCommMain-FileRecv: Failed to receive filename\n");
            return false;
        } else if ( recv_size==0 ) {
            usleep(1000);
            continue;
        }
        filename.append(pmySocket->bufRecv, recv_size);
        bytes_received += recv_size;
    }
    filename.resize(filename_len);
    printf("T1IPCCommMain-FileRecv: Receiving file: %s\n", filename.c_str());

    uint64_t file_size=0;
    bytes_received=0;
    char file_size_buf[8]={0};
    while ( bytes_received<8 ) {
        int recv_size = pmySocket->RecvData(client_fd, 1);
        if ( recv_size==-1 ) {
            printf("T1IPCCommMain-FileRecv: Failed to receive file size\n");
            return false;
        } else if ( recv_size==0 ) {
            usleep(1000);
            continue;
        }
        int copy_len = (bytes_received+recv_size)>8 ? (8-bytes_received) : recv_size;
        memcpy(file_size_buf+bytes_received, pmySocket->bufRecv, copy_len);
        bytes_received += copy_len;
    }
    memcpy(&file_size, file_size_buf, 8);
    file_size = ntohll(file_size);
    printf("T1IPCCommMain-FileRecv: File size: %lu bytes\n", (unsigned long)file_size);

    if ( file_size==0 || file_size>100*1024*1024 ) {
        printf("T1IPCCommMain-FileRecv: Invalid file size\n");
        return false;
    }

    std::ofstream out_file(save_path+filename, std::ios::out|std::ios::binary|std::ios::trunc);
    if ( !out_file.is_open() ) {
        printf("T1IPCCommMain-FileRecv: Failed to create file: %s%s\n", save_path.c_str(), filename.c_str());
        return false;
    }

    uint64_t total_received=0;
    while ( total_received<file_size ) {
        int recv_size = pmySocket->RecvData(client_fd, 1);
        if ( recv_size==-1 ) {
            printf("T1IPCCommMain-FileRecv: Connection closed unexpectedly\n");
            out_file.close();
            return false;
        } else if ( recv_size==0 ) {
            usleep(1000);
            continue;
        }

        int write_len = (total_received+recv_size)>file_size ? (file_size-total_received) : recv_size;
        out_file.write(pmySocket->bufRecv, write_len);
        total_received += write_len;

        float progress = (float)total_received/file_size*100;
        printf("\rT1IPCCommMain-FileRecv: Progress: %.1f%%", progress);
        fflush(stdout);
    }

    printf("\nT1IPCCommMain-FileRecv: File received: %s%s\n", save_path.c_str(), filename.c_str());
    out_file.close();

    received_filename = filename;
    return true;
}

int T1IPCCommMain::InitMainSide() {
    int debug=0;
    struct mq_attr attr;
    attr.mq_flags=0;
    attr.mq_maxmsg=10;
    attr.mq_msgsize=COMM_MSG_MAX_SIZE+1;
    attr.mq_curmsgs=0;

    mq_unlink(MAIN_TO_COMM_QUEUE);
    mq_unlink(COMM_TO_MAIN_QUEUE);

    mqMainToComm = mq_open(MAIN_TO_COMM_QUEUE, O_CREAT|O_WRONLY|O_NONBLOCK, 0666, &attr);
    if ( mqMainToComm==(mqd_t)-1 ) {
        if (debug>0) printf("T1IPCCommMain-InitMainSide: Failed to create main-to-comm queue: %s\n", strerror(errno));
        return -1;
    }

    mqCommToMain = mq_open(COMM_TO_MAIN_QUEUE, O_CREAT|O_RDONLY|O_NONBLOCK, 0666, &attr);
    if ( mqCommToMain==(mqd_t)-1 ) {
        if (debug>0) printf("T1IPCCommMain-InitMainSide: Failed to create comm-to-main queue: %s\n", strerror(errno));
        mq_close(mqMainToComm);
        mq_unlink(MAIN_TO_COMM_QUEUE);
        return -1;
    }

    if (debug>0) printf("T1IPCCommMain-InitMainSide: Main side IPC queues created.\n");
    return 0;
}

int T1IPCCommMain::InitCommSide() {
    int debug=0;
    mqMainToComm = mq_open(MAIN_TO_COMM_QUEUE, O_RDONLY | O_NONBLOCK);
    if ( mqMainToComm==(mqd_t)-1 ) {
        if (debug>0) printf("T1IPCCommMain-InitCommSide: Failed to open main-to-comm queue: %s\n", strerror(errno));
        return -1;
    }

    mqCommToMain = mq_open(COMM_TO_MAIN_QUEUE, O_WRONLY | O_NONBLOCK);
    if ( mqCommToMain==(mqd_t)-1 ) {
        if (debug>0) printf("T1IPCCommMain-InitCommSide: Failed to open comm-to-main queue: %s\n", strerror(errno));
        mq_close(mqMainToComm);
        mqMainToComm=(mqd_t)-1;
        return -1;
    }

    if (debug>0) printf("T1IPCCommMain-InitCommSide: Comm side IPC queues opened.\n");
    return 0;
}

void T1IPCCommMain::Cleanup() {
    if ( mqMainToComm!=(mqd_t)-1 ) {
        mq_close(mqMainToComm);
        mqMainToComm=(mqd_t)-1;
    }
    if ( mqCommToMain!=(mqd_t)-1 ) {
        mq_close(mqCommToMain);
        mqCommToMain=(mqd_t)-1;
    }
}

int T1IPCCommMain::SendToComm(char type, const char* data) {
    int debug=0;
    CommMessage msg;
    msg.type=type;
    if ( data ) {
        strncpy(msg.data, data, COMM_MSG_MAX_SIZE-1);
        msg.data[COMM_MSG_MAX_SIZE-1]='\0';
    } else {
        msg.data[0]='\0';
    }

    int ret = mq_send(mqMainToComm, (const char*)&msg, sizeof(msg), 0);
    if ( ret==-1 ) {
        if ( errno==EAGAIN ) {
            if (debug>0) printf("T1IPCCommMain-SendToComm: main-to-comm queue full, message dropped\n");
        } else {
            if (debug>0) printf("T1IPCCommMain-SendToComm: mq_send failed: %s\n", strerror(errno));
        }
        return -1;
    }
    return 0;
}

int T1IPCCommMain::RecvFromComm(char* data, int maxLen, int timeoutMs) {
    CommMessage msg;
    struct timespec ts;

    if ( timeoutMs>0 ) {
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += timeoutMs/1000;
        ts.tv_nsec += (timeoutMs%1000)*1000000L;
        if ( ts.tv_nsec>=1000000000L ) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000L;
        }
        int ret = mq_timedreceive(mqCommToMain, (char*)&msg, sizeof(msg), NULL, &ts);
        if ( ret==-1 ) {
            if ( errno==ETIMEDOUT ) return 0;
            return -1;
        }
    } else if ( timeoutMs==0 ) {
        int ret = mq_receive(mqCommToMain, (char*)&msg, sizeof(msg), NULL);
        if ( ret==-1 ) {
            if ( errno==EAGAIN ) return 0;
            return -1;
        }
    } else {
        int ret = mq_receive(mqCommToMain, (char*)&msg, sizeof(msg), NULL);
        if ( ret==-1 ) {
            return -1;
        }
    }

    if ( data ) {
        strncpy(data, msg.data, maxLen-1);
        data[maxLen-1]='\0';
    }
    return msg.type;
}

int T1IPCCommMain::SendToMain(char type, const char* data) {
    int debug=0;
    CommMessage msg;
    msg.type=type;
    if ( data ) {
        strncpy(msg.data, data, COMM_MSG_MAX_SIZE-1);
        msg.data[COMM_MSG_MAX_SIZE-1]='\0';
    } else {
        msg.data[0]='\0';
    }

    int ret = mq_send(mqCommToMain, (const char*)&msg, sizeof(msg), 0);
    if ( ret==-1 ) {
        if ( errno==EAGAIN ) {
            if (debug>0) printf("T1IPCCommMain-SendToMain: comm-to-main queue full, message dropped\n");
        } else {
            if (debug>0) printf("T1IPCCommMain-SendToMain: mq_send failed: %s\n", strerror(errno));
        }
        return -1;
    }
    return 0;
}

int T1IPCCommMain::RecvFromMain(char* data, int maxLen, int timeoutMs) {
    CommMessage msg;
    struct timespec ts;

    if ( timeoutMs>0 ) {
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += timeoutMs/1000;
        ts.tv_nsec += (timeoutMs%1000)*1000000L;
        if ( ts.tv_nsec>=1000000000L ) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000L;
        }
        int ret = mq_timedreceive(mqMainToComm, (char*)&msg, sizeof(msg), NULL, &ts);
        if ( ret==-1 ) {
            if ( errno==ETIMEDOUT ) return 0;
            return -1;
        }
    } else if ( timeoutMs==0 ) {
        int ret = mq_receive(mqMainToComm, (char*)&msg, sizeof(msg), NULL);
        if ( ret==-1 ) {
            if ( errno==EAGAIN ) return 0;
            return -1;
        }
    } else {
        int ret = mq_receive(mqMainToComm, (char*)&msg, sizeof(msg), NULL);
        if ( ret==-1 ) {
            return -1;
        }
    }

    if ( data ) {
        strncpy(data, msg.data, maxLen-1);
        data[maxLen-1]='\0';
    }
    return msg.type;
}

void T1IPCCommMain::Stop() {
    running=false;
}
