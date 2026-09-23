#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include <thread>
#include <chrono>
#include <algorithm>

#define DEFAULT_PORT    1111    //default port
#define BUFF_SIZE       1024    //buffer size
#define SELECT_TIMEOUT  100000       //select() timeout seconds

class T1Socket {
public:
    T1Socket();
    ~T1Socket();
    int GetSocket(char);
    int Select();
    int AcceptClient();
    int RecvData(int);
    int RecvData(int,int);
    int SendData(char *);
    int SendData2(const char *);
    int SendOneMapFile(const char *filename);

    int maxfd;
    fd_set readfds;
    char bufRecv[BUFF_SIZE]; 

private:
    fd_set readfds_bak; 
    struct timeval timeout;
   
    int serverSocket=0;
    int clientSocket=0;

    int InitSocket();
    void SetSockNonBlock(int);
    int UpdateMaxfd(fd_set fds, int maxfd);

};