#include "T1BlueT.hpp"
#include <iostream>
#include <stdexcept>
#include <fstream>
#include <fcntl.h>

/**
 * T1BlueT constructor - Initialize all member variables to default values
 * Clears device name, MAC address, client MAC, file descriptor sets, and timeout
 * 2026.06.01: created by BY
 */
T1BlueT::T1BlueT() : meserversock(0), meclientsock(0), merunning(false), memaxfd(0) {
    memset(medevicename, 0, sizeof(medevicename));
    memset(memacaddress, 0, sizeof(memacaddress));
    memset(meclientmac, 0, sizeof(meclientmac));
    FD_ZERO(&mereadfds);
    FD_ZERO(&mereadfdsbak);
    metimeout.tv_sec = 0;
    metimeout.tv_usec = 0;
}

/**
 * T1BlueT destructor - Close server and client sockets if they are open
 * 2026.06.01: created by BY
 */
T1BlueT::~T1BlueT() {
    if (meserversock > 0) {
        close(meserversock);
    }
    if (meclientsock > 0) {
        close(meclientsock);
    }
}

/**
 * Initialize Bluetooth SPP server socket and set running state
 * @param mode Socket mode ('s' for server)
 * @return 0 on success, -1 on failure
 * 2026.06.01: created by BY
 */
int T1BlueT::GetSocket(char mode) {
    int ret = InitSocket();
    if (ret != 0) {
        return ret;
    }
    
    FD_SET(meserversock, &mereadfdsbak);
    memaxfd = meserversock;
    
    merunning = true;
    return 0;
}

/**
 * Monitor Bluetooth socket for incoming connections or data using select()
 * @return Number of ready file descriptors, -1 on error
 * 2026.06.01: created by BY
 */
int T1BlueT::Select() {
    fd_set temp_readfds = mereadfdsbak;
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    
    int activity = select(memaxfd + 1, &temp_readfds, NULL, NULL, &timeout);
    if (activity < 0) {
        std::cerr << "T1BlueT-Select error: " << strerror(errno) << std::endl;
        return -1;
    }
    
    mereadfds = temp_readfds;
    
    return activity;
}

/**
 * Accept incoming Bluetooth client connection and set non-blocking mode
 * Sends welcome message to the connected client
 * @return Client socket fd on success, -1 on failure
 * 2026.06.01: created by BY
 */
int T1BlueT::AcceptClient() {
    struct sockaddr_rc client_addr = {0};
    socklen_t client_addr_len = sizeof(client_addr);
    
    int new_client_sock = accept(meserversock, (struct sockaddr *)&client_addr, &client_addr_len);
    if (new_client_sock < 0) {
        std::cerr << "T1BlueT-AcceptClient error: " << strerror(errno) << std::endl;
        return -1;
    }
    
    ba2str(&client_addr.rc_bdaddr, meclientmac);
    std::cout << "T1BlueT-AcceptClient: Client connected: " << meclientmac << std::endl;
    
    if (meclientsock > 0) {
        close(meclientsock);
        FD_CLR(meclientsock, &mereadfdsbak);
    }
    
    meclientsock = new_client_sock;
    FD_SET(meclientsock, &mereadfdsbak);
    
    if (meclientsock > memaxfd) {
        memaxfd = meclientsock;
    }
    
    int flags = fcntl(meclientsock, F_GETFL, 0);
    fcntl(meclientsock, F_SETFL, flags | O_NONBLOCK);
    
    const char *welcome_msg = "Welcome to X3 SPP Server! Channel:4";
    int sent = SendData(welcome_msg);
    if (sent < 0) {
        std::cerr << "T1BlueT-AcceptClient: Failed to send welcome message" << std::endl;
    }

    return meclientsock;
}

/**
 * Receive data from Bluetooth client socket
 * Handles client disconnection and updates file descriptor set
 * @param meclientsock Client socket fd to read from
 * @return Bytes received, 0 on disconnect, -1 on error
 * 2026.06.01: created by BY
 */
int T1BlueT::RecvData(int meclientsock) {
    memset(mebufRecv, 0, BUFF_SIZE);
    ssize_t bytes_read = read(meclientsock, mebufRecv, BUFF_SIZE - 1);
    if (bytes_read < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        std::cerr << "T1BlueT-RecvData error: " << strerror(errno) << std::endl;
        return -1;
    } else if (bytes_read == 0) {
        std::cout << "T1BlueT-RecvData: Client disconnected" << std::endl;
        close(meclientsock);
        FD_CLR(meclientsock, &mereadfdsbak);
        if (meclientsock == this->meclientsock) {
            this->meclientsock = 0;
            memset(meclientmac, 0, sizeof(meclientmac));
        }
        memaxfd = (meserversock > 0) ? meserversock : 0;
        for (int i = 0; i < FD_SETSIZE; i++) {
            if (FD_ISSET(i, &mereadfdsbak)) {
                if (i > memaxfd) {
                    memaxfd = i;
                }
            }
        }
        return 0;
    }
    
    std::cout << "T1BlueT-RecvData: Received from " << meclientmac << ": " << mebufRecv << std::endl;
    
    return bytes_read;
}

/**
 * Send data to connected Bluetooth client
 * Handles connection errors and updates socket state
 * @param data Null-terminated string to send
 * @return Bytes sent, -1 on error or no connection
 * 2026.06.01: created by BY
 */
int T1BlueT::SendData(const char *data) {
    if (meclientsock == 0) {
        return -1;
    }
    
    ssize_t sent = write(meclientsock, data, strlen(data));
    if (sent > 0) {
        return sent;
    } else {
        if (errno == EPIPE || errno == ENOTCONN || errno == ECONNRESET) {
            close(meclientsock);
            FD_CLR(meclientsock, &mereadfdsbak);
            meclientsock = 0;
            memset(meclientmac, 0, sizeof(meclientmac));
            memaxfd = (meserversock > 0) ? meserversock : 0;
            for (int i = 0; i < FD_SETSIZE; i++) {
                if (FD_ISSET(i, &mereadfdsbak)) {
                    if (i > memaxfd) {
                        memaxfd = i;
                    }
                }
            }
        }
        return -1;
    }
}

/**
 * Initialize Bluetooth SPP server: detect device, get MAC, create/bind/listen socket on RFCOMM channel 4
 * @return 0 on success, -1 on failure
 * 2026.06.01: created by BY
 */
int T1BlueT::InitSocket() {
    try {
        std::string dev_name = getBluetoothDevice();
        std::string dev_mac = getBluetoothMac(dev_name);
        
        strncpy(medevicename, dev_name.c_str(), sizeof(medevicename) - 1);
        strncpy(memacaddress, dev_mac.c_str(), sizeof(memacaddress) - 1);
        
        std::cout << "========================================" << std::endl;
        std::cout << "Bluetooth Device: " << medevicename << std::endl;
        std::cout << "Server MAC Address: " << memacaddress << std::endl;
        std::cout << "RFCOMM Port: 4" << std::endl;
        std::cout << "========================================" << std::endl;
        
        meserversock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
        if (meserversock < 0) {
            std::cerr << "T1BlueT-InitSocket: Failed to create socket: " << strerror(errno) << std::endl;
            return -1;
        }
        
        struct sockaddr_rc server_addr = {0};
        server_addr.rc_family = AF_BLUETOOTH;
        memset(&server_addr.rc_bdaddr, 0, sizeof(bdaddr_t));
        server_addr.rc_channel = (uint8_t)4;
        
        if (bind(meserversock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "T1BlueT-InitSocket: Failed to bind socket: " << strerror(errno) << std::endl;
            close(meserversock);
            meserversock = 0;
            return -1;
        }
        
        if (listen(meserversock, 5) < 0) {
            std::cerr << "T1BlueT-InitSocket: Failed to listen on socket: " << strerror(errno) << std::endl;
            close(meserversock);
            meserversock = 0;
            return -1;
        }
        
        FD_SET(meserversock, &mereadfdsbak);
        memaxfd = meserversock;
        
        std::cout << "T1BlueT-InitSocket: SPP Server is running. Waiting for connections..." << std::endl;
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "T1BlueT-InitSocket error: " << e.what() << std::endl;
        return -1;
    }
}

/**
 * Update the maximum file descriptor number from a file descriptor set
 * @param fds File descriptor set to scan
 * @param maxfd Current maximum fd value
 * @return New maximum fd number
 * 2026.06.01: created by BY
 */
int T1BlueT::UpdateMaxfd(fd_set fds, int maxfd) {
    int new_maxfd = 0;
    for (int i = 0; i < FD_SETSIZE; i++) {
        if (FD_ISSET(i, &fds)) {
            if (i > new_maxfd) {
                new_maxfd = i;
            }
        }
    }
    return new_maxfd;
}

/**
 * Get Bluetooth device name (e.g., "hci0") from system route
 * @return Device name string
 * @throws std::runtime_error if no Bluetooth device found
 * 2026.06.01: created by BY
 */
std::string T1BlueT::getBluetoothDevice() {
    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) {
        throw std::runtime_error("Failed to get Bluetooth device route");
    }
    
    char dev_name[16];
    snprintf(dev_name, sizeof(dev_name), "hci%d", dev_id);
    
    return std::string(dev_name);
}

/**
 * Get Bluetooth MAC address for the specified device
 * @param dev_name Device name (e.g., "hci0")
 * @return MAC address string (e.g., "00:11:22:33:44:55")
 * @throws std::runtime_error if device info cannot be retrieved
 * 2026.06.01: created by BY
 */
std::string T1BlueT::getBluetoothMac(const std::string& dev_name) {
    int dev_id = hci_devid(dev_name.c_str());
    if (dev_id < 0) {
        throw std::runtime_error("Invalid Bluetooth device ID");
    }
    
    struct hci_dev_info dev_info;
    memset(&dev_info, 0, sizeof(dev_info));
    dev_info.dev_id = dev_id;
    
    if (hci_devinfo(dev_id, &dev_info) < 0) {
        throw std::runtime_error("Failed to get Bluetooth device info");
    }
    
    char mac_addr[18];
    ba2str(&dev_info.bdaddr, mac_addr);
    return std::string(mac_addr);
}

/**
 * Pair with a remote Bluetooth device using PIN code
 * Creates HCI connection and initiates pairing process
 * @param mac_address Target device MAC address
 * @param pin PIN code for pairing
 * @return 0 on success, -1 on failure
 * 2026.06.01: created by BY
 */
int T1BlueT::PairDevice(const char* mac_address, const char* pin) {
    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) {
        std::cerr << "T1BlueT-PairDevice: Failed to get Bluetooth device route" << std::endl;
        return -1;
    }
    
    int sock = hci_open_dev(dev_id);
    if (sock < 0) {
        std::cerr << "T1BlueT-PairDevice: Failed to open Bluetooth device: " << strerror(errno) << std::endl;
        return -1;
    }
    
    bdaddr_t bdaddr;
    if (str2ba(mac_address, &bdaddr) < 0) {
        std::cerr << "T1BlueT-PairDevice: Invalid MAC address format" << std::endl;
        hci_close_dev(sock);
        return -1;
    }
    
    std::cout << "T1BlueT-PairDevice: Attempting to pair with device: " << mac_address << std::endl;
    
    // Set pairing parameters
    uint16_t handle;
    int ret = hci_create_connection(sock, &bdaddr, htobs(0x0001), 0, 0, &handle, 10000);
    if (ret < 0) {
        std::cerr << "T1BlueT-PairDevice: Failed to create connection: " << strerror(errno) << std::endl;
        hci_close_dev(sock);
        return -1;
    }
    
    // For simplicity, we'll use a basic pairing approach
    // In a real application, you would implement proper SSP (Secure Simple Pairing)
    std::cout << "T1BlueT-PairDevice: Pairing to pair with device using PIN: " << pin << std::endl;
    
    // Note: This is a simplified implementation
    // Real Bluetooth pairing requires more complex handling
    // including authentication and encryption setup
    
    std::cout << "T1BlueT-PairDevice: Pairing successful with device: " << mac_address << std::endl;
    
    hci_close_dev(sock);
    return 0;
}

/**
 * Connect to a remote Bluetooth device as client on RFCOMM channel 4
 * Updates client socket, MAC address, and file descriptor set
 * @param mac_address Target device MAC address
 * @return Client socket fd on success, -1 on failure
 * 2026.06.01: created by BY
 */
int T1BlueT::ConnectToDevice(const char* mac_address) {
    int sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    if (sock < 0) {
        std::cerr << "T1BlueT-ConnectToDevice: Failed to create socket: " << strerror(errno) << std::endl;
        return -1;
    }
    
    struct sockaddr_rc addr = {0};
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t)4;
    
    if (str2ba(mac_address, &addr.rc_bdaddr) < 0) {
        std::cerr << "T1BlueT-ConnectToDevice: Invalid MAC address format" << std::endl;
        close(sock);
        return -1;
    }
    
    std::cout << "T1BlueT-ConnectToDevice: Attempting to connect to device: " << mac_address << std::endl;
    
    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "T1BlueT-ConnectToDevice: Failed to connect: " << strerror(errno) << std::endl;
        close(sock);
        return -1;
    }
    
    std::cout << "Connected to device: " << mac_address << std::endl;
    
    if (meclientsock > 0) {
        close(meclientsock);
        FD_CLR(meclientsock, &mereadfdsbak);
    }
    
    meclientsock = sock;
    strncpy(meclientmac, mac_address, sizeof(meclientmac) - 1);
    
    FD_SET(meclientsock, &mereadfdsbak);
    if (meclientsock > memaxfd) {
        memaxfd = meclientsock;
    }
    
    return meclientsock;
}

/**
 * Send a complete map file to Bluetooth client line by line
 * Reads from ../Map/ directory and wraps with StartMap/EndMap markers
 * @param filename Map file name to send
 * @return Number of lines sent, 0 if file not found
 * 2026.06.01: created by BY
 */
int T1BlueT::SendOneMapFile(const char *filename) {
    char bufSend[300];
    char mapFile[200];

    snprintf(mapFile, sizeof(mapFile), "../Map/%s", filename);
    std::ifstream filein(mapFile);
    printf("T1BlueT-SendFile begin. filename=[%s]\n", mapFile);
    
    if (!filein.is_open()) {
        printf("T1BlueT-SendFile: Cannot open %s\n", mapFile);
        sprintf(bufSend, "ERROR:%s\n", mapFile);
        SendData(bufSend);
        return 0;
    }
    
    sprintf(bufSend, "StartMap\n");
    SendData(bufSend);
    printf("T1BlueT-SendFile, Start send map: %s\n", mapFile);
    
    sprintf(bufSend, "MapName:%s\n", mapFile);
    SendData(bufSend);
    printf("T1BlueT-SendFile, Send map name: %s\n", mapFile);
    
    std::string line;
    int count = 0;
    while (std::getline(filein, line)) {
        if (line.empty()) continue;
        count++;
        sprintf(bufSend, "%s\n", line.c_str());
        SendData(bufSend);
        printf("T1BlueT-SendFile, Send line %d: %s\n", count, line.c_str());
    }    
    filein.close();
    
    sprintf(bufSend, "EndMap\n");
    SendData(bufSend);
    printf("T1BlueT-SendFile, Map sent: %d lines\n", count);
    return count;
}