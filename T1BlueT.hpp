#ifndef T1BLUET_HPP
#define T1BLUET_HPP

/* ============================================================
 * BUILD SHIM - this file belongs to the Tuya staging app only.
 *
 * T1CommMain.cpp is carried over verbatim from the1Comm and includes this
 * header, but the BLE SPP channel is not part of this build: its .cpp is not
 * carried over (it would also need libbluetooth), so the class is a no-op
 * stand-in. It lets the original main compile and link unchanged while the
 * channel that really runs is the Tuya one.
 *
 * Do NOT copy this file back into the1Comm: the repository has the real
 * T1BlueT.hpp plus T1BlueT.cpp.
 *
 * meserversock makes the difference in the event loop: it starts at -1, so
 *     if ( blueT.meserversock>0 )   and   blueT.Select()
 * are both skipped and the stand-in is never asked for data.
 * 2026.09.21: created by Lff（+AI）, build shim for the Tuya staging app
 * ============================================================ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <string>

#define BUFF_SIZE 1024    /* buffer size, same value as the1Comm */

class T1BlueT {
public:
    T1BlueT() : memaxfd(0), meserversock(-1), meclientsock(-1), merunning(false) {
        FD_ZERO(&mereadfds);
        mereadfdsbak=mereadfds;
        memset(mebufRecv, 0, sizeof(mebufRecv));
        memset(meclientmac, 0, sizeof(meclientmac));
    }
    ~T1BlueT() {}

    int GetSocket(char mode) { (void)mode; return -1; }
    int Select() { return 0; }
    int AcceptClient() { return -1; }
    int RecvData(int fd) { (void)fd; return -1; }
    int SendData(const char *data) { (void)data; return -1; }
    int SendOneMapFile(const char *filename) { (void)filename; return -1; }

    /* pairing is done by the Tuya BLE netcfg service in this build */
    int PairDevice(const char *mac, const char *pin) { (void)mac; (void)pin; return -1; }
    int ConnectToDevice(const char *mac) { (void)mac; return -1; }

    int memaxfd;
    fd_set mereadfds;
    char mebufRecv[BUFF_SIZE];
    int meserversock;
    int meclientsock;
    char meclientmac[18];
    bool merunning;

private:
    fd_set mereadfdsbak;
    std::string medevicename;
    std::string memacaddress;
};

#endif
