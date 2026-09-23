#ifndef T1SOCKET_HPP
#define T1SOCKET_HPP

/* ============================================================
 * BUILD SHIM - this file belongs to the Tuya staging app only.
 *
 * T1CommMain.cpp is carried over verbatim from the1Comm and includes this
 * header, but the 1111 socket channel is not part of this build: its .cpp is
 * not carried over, so the class is a no-op stand-in. It lets the original
 * main compile and link unchanged while the channel that really runs is the
 * Tuya one.
 *
 * Do NOT copy this file back into the1Comm: the repository has the real
 * T1Socket.hpp plus T1Socket.cpp.
 *
 * What the stand-in answers, which is what keeps the event loop quiet:
 *     GetSocket() -> -1, no listen socket is opened, blueOk turns false
 *     Select()    ->  0, never an event, so no branch of the loop is entered
 * 2026.09.21: created by Lff（+AI）, build shim for the Tuya staging app
 * ============================================================ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <unistd.h>

#define DEFAULT_PORT    1111    /* default port, same value as the1Comm */
#define BUFF_SIZE       1024    /* buffer size, same value as the1Comm */
#define SELECT_TIMEOUT  100000  /* select() timeout, same value as the1Comm */

class T1Socket {
public:
    T1Socket() : maxfd(0), serverSocket(-1), clientSocket(-1) {
        FD_ZERO(&readfds);
        readfds_bak=readfds;
    }
    ~T1Socket() {}

    int GetSocket(char mode) { (void)mode; return -1; }
    int Select() { return 0; }
    int AcceptClient() { return -1; }
    int RecvData(int fd) { (void)fd; return -1; }
    int RecvData(int fd, int len) { (void)fd; (void)len; return -1; }
    int SendData(char *data) { (void)data; return -1; }
    int SendData2(const char *data) { (void)data; return -1; }
    int SendOneMapFile(const char *filename) { (void)filename; return -1; }

    int maxfd;
    fd_set readfds;
    char bufRecv[BUFF_SIZE];

private:
    fd_set readfds_bak;
    int serverSocket;
    int clientSocket;
};

#endif
