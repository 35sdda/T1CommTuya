#ifndef T1BLUET_HPP
#define T1BLUET_HPP

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <sys/select.h>
#include <time.h>
#include <string>
#include <vector>

#define BUFF_SIZE 1024    //buffer size

class T1BlueT {
public:
    T1BlueT();
    ~T1BlueT();
    
    int GetSocket(char mode);
    int Select();
    int AcceptClient();
    int RecvData(int meclientsock);
    int SendData(const char *data);
    
    // Pairing and connection functions
    int PairDevice(const char* mac_address, const char* pin);
    int ConnectToDevice(const char* mac_address);
    int SendOneMapFile(const char *filename);
    int SendLogList(const std::vector<std::string>& logList);
    
    int memaxfd;
    fd_set mereadfds;
    char mebufRecv[BUFF_SIZE];
    int meserversock;
    int meclientsock;
    char meclientmac[18];
    bool merunning;
    
private:
    fd_set mereadfdsbak;
    struct timeval metimeout;
    
    char medevicename[16];
    char memacaddress[18];
    
    int InitSocket();
    int UpdateMaxfd(fd_set fds, int maxfd);
    std::string getBluetoothDevice();
    std::string getBluetoothMac(const std::string& dev_name);
};

#endif // T1BLUET_H
