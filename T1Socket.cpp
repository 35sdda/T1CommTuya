#include "T1Socket.hpp"
#include <fstream>
#include <string>
#include <dirent.h>

T1Socket::T1Socket() {
    if ( (serverSocket = socket(PF_INET, SOCK_STREAM, 0)) == -1 ) {
        perror("T1Socket: T1Socket socket failed, ");
        exit(EXIT_FAILURE);
    }
    //printf("T1Socket step 1, socket done\n");
    
    if ( InitSocket() < 0 ) {
        close(serverSocket);
        perror("T1Socket: Initialize server socker error.\n");
        exit(EXIT_FAILURE);
    }
    
    // initialize arguments for select() (only use read)??and add socket in fd_set array
    maxfd = serverSocket;
    FD_ZERO(&readfds);
    FD_ZERO(&readfds_bak);  //backup for readfds(becaue select() will refresh readfds??so we must backup it)
    FD_SET(serverSocket, &readfds_bak);

    printf("T1Socket: serverSocket initilizeed.\n");
}

T1Socket::~T1Socket() {
    if ( serverSocket != 0 )
        close(serverSocket);
    if ( clientSocket != 0 )
        close(clientSocket);
}

// initialize the server socker
int T1Socket::InitSocket() {
    //in case of 'address already in use' error message
    int yes = 1;
    if (setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int))) {
        perror("T1Socket-InitSocket(): setsockopt failed");
        return -1;
    }

    // set socket tp non-blocking
    SetSockNonBlock(serverSocket);

    // creat socket address for bind
    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // can accept any sddress
    bind_addr.sin_port = htons(DEFAULT_PORT);       // change host byte order to network byte order

    //bind socket to the address above
    if ( bind(serverSocket, (struct sockaddr *) &bind_addr, sizeof(bind_addr)) == -1 ) {
        perror("T1Socket-InitSocket(): bind failed, ");
        return -2;
    }
    //printf("T1Socket step 2, bind done\n");

    //listen
    if ( listen(serverSocket, 5) == -1) {
        perror("T1Socket-InitSocket() listen failed.");
        return -3;
    }
    //printf("T1Socket step 3, listen done\n");

    return 1;
}

int T1Socket::GetSocket(char type) { // 's' for server, 'c' for client
    if ( type=='s' )
        return serverSocket;
    else if ( type=='c' )
        return clientSocket;
    else
        return 0;
}

// set socket to non-blocking mode
void T1Socket::SetSockNonBlock(int sock) {
    int flags;
    flags = fcntl(sock, F_GETFL, 0);
    if (flags < 0) {
        perror("T1Socket-SetSockNonBlock(): fcntl(F_GETFL) failed");
        exit(EXIT_FAILURE);
    }
    if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0) {
        perror("T1Socket-SetSockNonBlock(): fcntl(F_SETFL) failed");
        exit(EXIT_FAILURE);
    }
}

// select() check all file directorys
int T1Socket::Select() {
    // after select(), readfds and timeout will all be changed, so we must reset them
    readfds = readfds_bak;
    maxfd = UpdateMaxfd(readfds, maxfd);        // refresh maxfd
    timeout.tv_sec = 0;
    timeout.tv_usec = 0; // return immediatelly

    //printf("waiting, select() maxfd=%d\n", maxfd);
    //select(), we don't need write_fds and error_fds??so they are NULL
    int ret = select(maxfd + 1, &readfds, NULL, NULL, &timeout);
    if ( ret==-1 ) {
        perror("T1Socket-Select(): select failed");
        return -1; //exit(EXIT_FAILURE);
    } else if ( ret==0 ) {
        //fprintf(stderr, "no socket ready for read within %d microsecs\n", SELECT_TIMEOUT);
        return 0; //continue;
    }
    
    //printf("T1Socket-Select(): select() maxfd=%d, ret=%d\n", maxfd, ret);
    return ret;
}

// calculate the max file directory
int T1Socket::UpdateMaxfd(fd_set fds, int maxfd) {
    int i;
    int new_maxfd = 0;
    for (i = 0; i <= maxfd; i++) {
        if (FD_ISSET(i, &fds) && i > new_maxfd) {
            new_maxfd = i;
        }
    }
    return new_maxfd;
}

// accept a connection from client
int T1Socket::AcceptClient() {
    //int new_sock;
    struct sockaddr_in client_addr;
    socklen_t client_addr_len;
    client_addr_len = sizeof(client_addr);
    
    if ( clientSocket != 0 )
        close(clientSocket);
    clientSocket = accept(serverSocket, (struct sockaddr *) &client_addr, &client_addr_len);
    if (clientSocket == -1) {
        perror("T1Socket-AcceptClient(): accept failed");
        return -1;//exit(EXIT_FAILURE);
    }
    /*if (!inet_ntop(AF_INET, &(client_addr.sin_addr), client_ip_str, sizeof(client_ip_str))) {
        perror("inet_ntop failed");
        exit(EXIT_FAILURE);
    }*/
    printf("T1Socket-AcceptClient(): accept a client from: %d\n", clientSocket);//client_ip_str);
    //set the new clientSocket to non-blocking
    SetSockNonBlock(clientSocket);
    //add the new clientSocket in the select()'s monitor
    if (clientSocket > maxfd) {
        maxfd = clientSocket;
    }
    FD_SET(clientSocket, &readfds_bak);

    return 0;
}

// receive data from client
int T1Socket::RecvData(int client) {
    memset(bufRecv, 0, sizeof(bufRecv));
    int recv_size = recv(client, bufRecv, sizeof(bufRecv), 0);
    if (recv_size == -1) {
        perror("T1Socket-RecvData(): recv failed");
        return -1;
    } else if (recv_size == 0) {
        perror("T1Socket-RecvData(): Client closed the connection.");
        close(client);
        FD_CLR(client, &readfds_bak);
        clientSocket = 0; 
        printf("T1Socket-RecvData(): Waiting for new client connection...\n");
        return 0;
    } else {
        //printf("T1Socket-RecvData(), recved from new_sock=%d : %s(%d length string)\n", i, buffer, recv_size);
        return recv_size;
    }
}

// send data to client
int T1Socket::SendData(char *buff) {
    int ret;
    //printf("sending %s, %d\n", buff, clientSocket);
    ret = send( clientSocket, buff, strlen(buff), MSG_NOSIGNAL );
    if ( ret < 0 ) {
        ;//printf("send error, %d\n",ret);
    } else {
        //printf("T1Socket-SendData(): send %d bytes:(%s)\n", ret, buff);
    }
    return ret;
}

int T1Socket::RecvData(int client,int a) {
    memset(bufRecv, 0, sizeof(bufRecv));
    int recv_size = recv(client, bufRecv, sizeof(bufRecv), 0);
    if (recv_size == -1) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0; 
        } else {
            perror("T1Socket-RecvData(): recv failed");
            return -1;
        }
    } else if (recv_size == 0) {
        perror("T1Socket-RecvData(): Client closed the connection.");
        close(client);
        FD_CLR(client, &readfds_bak);
        clientSocket = 0; 
        printf("T1Socket-RecvData(): Waiting for new client connection...\n");
        return 0;
    } else {
        //printf("T1Socket-RecvData(), recved from new_sock=%d : %s(%d length string)\n", i, buffer, recv_size);
        return recv_size;
    }
}

int T1Socket::SendData2(const char *buff) {
    int total_sent = 0;
    int data_length = strlen(buff);
    int ret;
    const int chunk_size = 512; 
    
    printf("T1Socket-SendData2(): Start sending %d bytes in chunks of %d bytes\n", data_length, chunk_size);
    
    while (total_sent < data_length) {
        int send_size = std::min(chunk_size, data_length - total_sent);
        ret = send(clientSocket, buff + total_sent, send_size, MSG_NOSIGNAL);
        if (ret < 0) {
            perror("T1Socket-SendData2(): send failed");
            return -1;
        }
        total_sent += ret;
        printf("T1Socket-SendData2(): Sent chunk of %d bytes, total %d/%d\n", ret, total_sent, data_length);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    printf("T1Socket-SendData2(): Send complete, total %d bytes\n", total_sent);
    return total_sent;
}

/**
 * Send map file data to App
 */
int T1Socket::SendOneMapFile(const char *filename) {
    char bufSend[300];
    char mapFile[200];

    snprintf(mapFile, sizeof(mapFile), "../Map/%s", filename);
    std::ifstream filein(mapFile);
    printf("T1Socket-SendFile begin. filename=[%s]\n", mapFile);
    
    if (!filein.is_open()) {
        printf("T1Socket-SendFile: Cannot open %s\n", mapFile);
        sprintf(bufSend, "ERROR:%s\n", mapFile);
        SendData(bufSend);
        return 0;
    }
    
    // send begin signal
    sprintf(bufSend, "StartMap\n");
    SendData(bufSend);
    printf("T1Socket-SendFile, Start send map: %s\n", mapFile);
    
    // send map filename
    sprintf(bufSend, "MapName:%s\n", mapFile);
    SendData(bufSend);
    printf("T1Socket-SendFile, Send map name: %s\n", mapFile);
    
    // send each lines
    std::string line;
    int count = 0;
    while (std::getline(filein, line)) {
        if (line.empty()) continue;
        count++;
        sprintf(bufSend, "%s\n", line.c_str());
        SendData(bufSend);
        printf("T1Socket-SendFile, Send line %d: %s\n", count, line.c_str());
    }    
    filein.close();
    
    // send end signal
    sprintf(bufSend, "EndMap\n");
    SendData(bufSend);
    printf("T1Socket-SendFile, Map sent: %d lines\n", count);
    return count;
}
