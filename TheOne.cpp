#include "TheOne.hpp"
#include "T1Utils.hpp"

int main()
{
    printf("TheOne-main begin.....\n");
    //CheckEnviroment();

    T1Socket mySocket; 
    char bufSocketSend[BUFF_SIZE];
    
    T1BlueT myBlueT;
    char bufBlueSend[BUFF_SIZE];
    
    Serial myCom;
    //unsigned char bufSerialSend[] = "$V0,0,2,3,4,5,7654321\r\n" ;
    char bufSerialSend[200] = "$V0,0,2,3,4,5,7654321\r\n" ; 
    int lengthSend=0; // length of sstring sent by serial port
    //char bufSerialSend[] = "1234567890000000000\r\n" ;
    unsigned char bufSerialRecv[200] = {0};
    
    T1All myAll;    
    int testtype = myAll.GetSignal('N');
    
    //std::string str = "/dev/ttyS2";   // miniboard
    //std::string str = "/dev/ttyS3";   // wave-share
    std::string str = myAll.GetString('S'); // setting in WhoIsRobo.upt
    printf("TheOne-main, serial port is: [%s]\n", str.c_str());    
    myCom.OpenSerial(str, E_BaudRate::_115200, E_DataSize::_8, E_Parity::NoneParity, E_StopBit::_1);
    
    /******** init bluetooth socket ********/
    int blueRet = myBlueT.GetSocket('s');
    if (blueRet != 0) {
        printf("TheOne-main: Bluetooth init failed\n");
    } else {
        printf("TheOne-main: Bluetooth SPP Server initialized, meserversock=%d\n", myBlueT.meserversock);
    }
    
    /******** send factory ********/
    /*
    int lengthSend=0; // length of sstring sent by serial port
    sprintf(bufSerialSend,"@CMD,%s\r\n", myAll.GetString('F'));
    lengthSend = myCom.Send( (unsigned char*)bufSerialSend, strlen(bufSerialSend) );
    printf("TheOne-main SendFactory:%d,[%s]\n\n\n\n\n", lengthSend, bufSerialSend);
    if ( lengthSend>=0 ) bufSerialSend[0] = 0;
    */

    /******** prepare time count ****/
    int count=0, timepause=100;
    uint64_t timenow = T1Utils::getCurrentTimeUs();
    uint64_t time0 = timenow;
    uint64_t time1 = time0;
    
    while (true) { 
//    for ( int i=0; i<150; i++ ) {
        count ++;
        if ( testtype == 2 ) printf("\nTheOne-main Loop(%d), ", count);
        timepause = 10 - timeCalculate(testtype, &time0, &timenow, "0");
        
        /*********** usleep to set loop time to 10ms ******/
        //printf(", timepause = %d\n", timepause);
        if ( timepause < 0 ) timepause = 0;
        if ( timepause > 10 ) timepause = 10; // Limit sleep to 10ms max
        usleep(timepause*1000);
        timeCalculate(testtype, &time0, &timenow, "usleep");
        time1 = time0;

        /********** Drawer action *****/
        if ( count%10==0 ) {
            myAll.Draw();
            //strcpy(bufSerialSend,"$V0,0,2,3,4,5,7654321\r\n");
            int key = myAll.GetSignal('K');
            if ( key == 'o' ) {
                sprintf(bufSerialSend,"@CMD,G\r\n"); // stop all before exit
                lengthSend = myCom.Send( (unsigned char*)bufSerialSend, strlen(bufSerialSend) );
                printf("TheOne-main, exit when 'o' pressed.\n");
                myAll.ProcessVision('C'); // vision close the FIFO
                //exit(0);
                break;
            } else if ( key != 0 ) {
                std::string cmdStr = tranCmdStr(&myAll,'K',key);
                strcpy(bufSerialSend,cmdStr.c_str());
                myAll.SetStatus('K',0); // clean the KeySignal, only use once
                printf("TheOne-main, [%c] tranCmdStr ended.\n", key);
            } else {
            }
        }
        timeCalculate(testtype, &time1, &timenow, "maAll.Draw()");
        //continue;

         /*********** Vision detect process ******/
        if ( count%1 == 0 ) {
            PointRobo currentData = myAll.ProcessVision('D');
            //printf("TheOne-main,[10ms] Running | xyyaw=(%.0f,%.0f,%d)\n",currentData.fx,currentData.fy,currentData.yaw);
        }
        timeCalculate(testtype, &time1, &timenow, "myVision");
//        continue;

        /*********** serial send **********/
        if ( bufSerialSend[0] == '@' ) { // command, send immediately
            lengthSend = myCom.Send( (unsigned char*)bufSerialSend, strlen(bufSerialSend) );            
            printf("TheOne-main serial (%s) send: byte[%d], key[%s]\n", str.c_str(), lengthSend, bufSerialSend);
            if ( lengthSend>0 ) bufSerialSend[0] = 0;
        } else if ( count % 10 == 0 ) { // status, send 1 per second
            sprintf(bufSerialSend,"$V0,0,2,3,4,5,7654321\r\n");
            lengthSend = myCom.Send( (unsigned char*)bufSerialSend, strlen(bufSerialSend) );
            //printf("T1main 1: %s send bytes(%d),[%s]\n", str.c_str(), lengthSend, bufSerialSend);
            if ( lengthSend>0 ) bufSerialSend[0] = 0;
        }
        timeCalculate(testtype, &time1, &timenow, "serial send");
//        continue;

        /*********** serial recive **********/
        lengthSend = myCom.Recv(bufSerialRecv, sizeof(bufSerialRecv));
        //std::cout << "lengthSend = " << lengthSend << std::endl;
        if ( lengthSend > 0 ) {
            //printf("T1main: ");//%s", bufSerialRecv);
            int retFirm = myAll.ProcessFirmSerial(lengthSend,bufSerialRecv);
            if ( retFirm==-1 ) { // reset 415
                sprintf(bufSerialSend,"@CMD,G\r\n");
                lengthSend = myCom.Send( (unsigned char*)bufSerialSend, strlen(bufSerialSend) );
                sprintf(bufSerialSend,"@CMD,R\r\n");
                lengthSend = myCom.Send( (unsigned char*)bufSerialSend, strlen(bufSerialSend) );
                if ( lengthSend>0 ) bufSerialSend[0] = 0;
                printf("TheOne-main Reset 415!!!!:%d\n\n\n\n\n", lengthSend);
            } else if ( retFirm==-2 ) { // need send Factory
                sprintf(bufSerialSend,"@CMD,%s\r\n", myAll.GetString('F'));
                lengthSend = myCom.Send( (unsigned char*)bufSerialSend, strlen(bufSerialSend) );
                printf("TheOne-main SendFactory:%d,[%s]\n\n\n\n\n", lengthSend, bufSerialSend);
                if ( lengthSend>=0 ) bufSerialSend[0] = 0;
            } else { // normal, success
                ;
            }
        }

        timeCalculate(testtype, &time1, &timenow, "serial recive");
//        continue;

        /*********** F415 key signal **********/
        int key415 = myAll.GetSignal('F');
        if ( key415 != 0 ) {
            std::string cmdStr = tranCmdStr(&myAll, 'K', key415);
            strcpy(bufSerialSend, cmdStr.c_str());
            myAll.SetStatus('F', 0); // clean the F415 KeySignal, only use once
            printf("TheOne-main, F415 key [%c] tranCmdStr ended.\n", key415);
        }
        timeCalculate(testtype, &time1, &timenow, "Robot Act2");
//        continue;

        /********* Robot Act immediately after Serial Sensor **********/
        myAll.Work(); 
        timeCalculate(testtype, &time1, &timenow, "Robot Act1");
        /*********** serial send **********/
        sprintf(bufSerialSend,"%s", myAll.GetString('C'));
        sprintf(bufSocketSend, "%s", bufSerialSend);
        myAll.SetStatus('C',0); // clean the CmdSend, only use once
        if ( bufSerialSend[0] == '@' ) {
            lengthSend = myCom.Send( (unsigned char*)bufSerialSend, strlen(bufSerialSend) );
            //printf("T1main RoboWork-CmdSend: byte[%d], [%s]\n", lengthSend, bufSerialSend);
            if ( lengthSend>0 ) bufSerialSend[0] = 0;
        } else {
            if ( strlen(bufSerialSend) > 0 )
                printf("TheOne-main RoboWork, not '@' begin:%d,[%s]\n", lengthSend, bufSerialSend);
        }
        
        /*********** bluetooth send **********/
        if (myBlueT.meclientsock > 0) {
            sprintf(bufBlueSend, "%s", myAll.GetString('O'));
            if ( bufBlueSend[0]=='@' ) {
                lengthSend = myBlueT.SendData(bufBlueSend);
                if ( lengthSend>0 ) {
                    bufBlueSend[0] = 0;
                    myAll.SetStatus('O',0);
                }
            } else if ( count%25==0 ) {
                std::string HeartBeatStr = myAll.getHeartBeatStr();
                strcpy(bufBlueSend, HeartBeatStr.c_str());
                myBlueT.SendData(bufBlueSend);
            }
        }
        //printf("T1main: Send.\n");
        timeCalculate(testtype, &time1, &timenow, "socket send");
//        continue;

        /*********** bluetooth receive **********/
        if (myBlueT.meserversock > 0) {
            int blueRet = myBlueT.Select();
            timeCalculate(testtype, &time1, &timenow, "bluetooth select");
            
            if (blueRet > 0) {
                for (int i = 0; i <= myBlueT.memaxfd; i++) {
                    if (!FD_ISSET(i, &(myBlueT.mereadfds))) {
                        continue;
                    }
                    if (i == myBlueT.meserversock) {
                        if (myBlueT.AcceptClient() < 0) {
                            printf("TheOne-main: Failed to accept bluetooth client connection.\n");
                            break;
                        }
                        printf("TheOne-main: bluetooth receive data......\n");
                    } else {
                        int recv_size = myBlueT.RecvData(i);
                        if (recv_size > 3 && myBlueT.mebufRecv[0] == '$' && myBlueT.mebufRecv[2] == ':') {
                            if (myBlueT.mebufRecv[1] == 'C') {
                                printf("TheOne-main: received from Bluetooth: [%s] (length=%d)\n", myBlueT.mebufRecv, recv_size);
                                strcpy(bufSerialSend, tranCmdStr(&myAll, 'S', myBlueT.mebufRecv[3]).c_str());
                                lengthSend = myCom.Send((unsigned char*)bufSerialSend, strlen(bufSerialSend));
                                printf("TheOne-main 2: %s send bytes: %d\n", bufSerialSend, lengthSend);
                            } else if (myBlueT.mebufRecv[1] == 'm') {
                                printf("TheOne-main: received from Bluetooth: [%s] (length=%d)\n", myBlueT.mebufRecv, recv_size);
                                myAll.MapSwitchNew(myBlueT.mebufRecv + 4);
                            } else if (myBlueT.mebufRecv[1] == 'T') {
                                if (isTimeSetOk != 0) {
                                    isTimeSetOk = myAll.SetSysTime(myBlueT.mebufRecv, recv_size);
                                }
                            } else if (myBlueT.mebufRecv[1] == 'd') {
                                printf("TheOne-main: received from Bluetooth: [%s] (length=%d)\n", myBlueT.mebufRecv, recv_size);
                                printf("TheOne-main: Delete Map command via Bluetooth\n");
                                myAll.MapDelete(myBlueT.mebufRecv + 3);
                            } else if (myBlueT.mebufRecv[1] == 'f') {
                                printf("TheOne-main: received from Bluetooth: [%s] (length=%d)\n", myBlueT.mebufRecv, recv_size);
                                printf("TheOne-main: Send Map Content command via Bluetooth\n");
                                myBlueT.SendOneMapFile(myBlueT.mebufRecv + 4);
                            } else if (myBlueT.mebufRecv[1] == 'l') {
                                printf("TheOne-main: received from Bluetooth: [%s] (length=%d)\n", myBlueT.mebufRecv, recv_size);
                                printf("TheOne-main: Send Map Name command via Bluetooth\n");
                                char mapNameMsg[] = "$MapName\n";
                                myBlueT.SendData(mapNameMsg);
                                std::string mapListJson = myAll.MapGetList();
                                myBlueT.SendData((char*)mapListJson.c_str());
                            } else if (myBlueT.mebufRecv[1] == 'M') {
                                printf("TheOne-main: received from Bluetooth: [%s] (length=%d)\n", myBlueT.mebufRecv, recv_size);
                                myAll.MapRecvLine(myBlueT.mebufRecv, "../Map/");
                            } else if ( myBlueT.mebufRecv[1] == 'I' ) {
                                printf("TheOne-main: received from Bluetooth,Request all logs: [%s] (length=%d)\n", myBlueT.mebufRecv, recv_size);
                                myBlueT.SendLogList(myAll.GetLogList());
                            } else {
                                printf("TheOne-main: received from Bluetooth: [%s] (length=%d)\n", myBlueT.mebufRecv, recv_size);
                                printf("TheOne-main bluetooth received error: [%s]\n", myBlueT.mebufRecv);
                            }
                        }
                    }
                }
            }
            timeCalculate(testtype, &time1, &timenow, "bluetooth receive");
        }

        /*********** socket send **********/
        /** send status or files to APP **/
        sprintf(bufSocketSend, "%s", myAll.GetString('O'));
        if ( bufSocketSend[0] == '@' ) { // send status
            //printf("TheOne-main send status.\n");
            lengthSend = mySocket.SendData(bufSocketSend);
            if ( lengthSend>0 ) {
                bufSocketSend[0] = 0;
                myAll.SetStatus('O',0); // clean the SocketSignal, only use once
            }
        } else if ( bufSocketSend[0] == 'F' ) { // send files
            //printf("TheOne-main send file.\n");
            //lengthSend = FileSendToApp(&mySocket, bufSocketSend);
            if ( lengthSend>0 ) {
                bufSocketSend[0] = 0;
                myAll.SetStatus('O',0); // clean the SocketSignal, only use once
            }
        } else if (count % 25 == 0){
            //printf("TheOne-main send heatdbeat.\n");
            std::string HeartBeatStr = myAll.getHeartBeatStr();//&myAll);
            strcpy(bufSocketSend,HeartBeatStr.c_str());
            mySocket.SendData(bufSocketSend);
        }

        /*********** socket receive **********/
        int ret = mySocket.Select();
        //printf("T1main: while-loop, mySocket.ret=%d\n", ret);
        timeCalculate(testtype, &time1, &timenow, "socket select");
//        continue;
        if ( ret == 0 ) {
            //printf("TheOne-main: socket select(), nothing to recive, continue loop.\n");
            continue;
            //mySocket.maxfd = -1;
        } else if ( ret < 0 ) {
            printf("TheOne-main: Some error when select()\n");
            exit(EXIT_FAILURE);
        }
        
        /** check all sockets if is serverSocket then accept, if is clientSocket then recv */
        //printf("T1main, now to receive socket.\n"); 
        for ( int i=0; i<=mySocket.maxfd; i++ ) {
            //printf("T1main: for-loop to check socket ..%d..\n",i);
            if ( !FD_ISSET(i, &(mySocket.readfds)) ) {
                continue; // only process flaged socket
                //break; // only process flaged socket
            }
            if ( i == mySocket.GetSocket('s')) {
                // if is serverSocket then accept()
                if ( mySocket.AcceptClient() < 0 ) {
                    printf("TheOne-main: Failed to accept client connection.\n");
                    //break;
                    continue;
                }
                printf("TheOne-main: receive data......\n");
            } else {
                /** if is clientSocket then recv(), we can read and write */
                int recv_size = mySocket.RecvData(i);
                //if ( recv_size > 0 ) {
                if ( recv_size>3 && mySocket.bufRecv[0]=='$' && mySocket.bufRecv[2]==':') {
                    //printf("TheOne-main: received from Android: [%s] (length=%d)\n", mySocket.bufRecv, recv_size);
                    if ( mySocket.bufRecv[1] == 'C' ) { // tranfor to serial port F415
                        printf("TheOne-main: received from Android: [%s] (length=%d)\n", mySocket.bufRecv, recv_size);
                        //sprintf(bufSerialSend,"@CMD,%c\r\n",mySocket.bufRecv[0]);
                        strcpy( bufSerialSend, tranCmdStr(&myAll,'S', mySocket.bufRecv[3]).c_str() );
                        lengthSend = myCom.Send( (unsigned char*)bufSerialSend, strlen(bufSerialSend) );
                        printf("TheOne-main 2: %s send bytes: %d\n", bufSerialSend, lengthSend);
                    } else if ( mySocket.bufRecv[1] == 'm' ) { // new, change sellected map by filename
                        printf("TheOne-main: received from Android: [%s] (length=%d)\n", mySocket.bufRecv, recv_size);
                        myAll.MapSwitchNew(mySocket.bufRecv + 4); // +3 or +4, need to test     
                    } else if ( mySocket.bufRecv[1] == 'T' ) { // time sync from App: $T:yyyyMMddHHmmss
                        //printf("TheOne-main: received from Android: [%s] (length=%d)\n", mySocket.bufRecv, recv_size);
                        if ( isTimeSetOk != 0 ) {
                            isTimeSetOk = myAll.SetSysTime(mySocket.bufRecv, recv_size);
                        }
                    } else if ( mySocket.bufRecv[1] == 'F' ) { // receive file
                        printf("TheOne-main: received from Android: [%s] (length=%d)\n", mySocket.bufRecv, recv_size);
                        printf("TheOne-main: Received file transfer command\n");
                        // Check if client socket is valid
                        int client_fd = mySocket.GetSocket('c');
                        if (client_fd > 2) { // Valid file descriptors are typically greater than 2 (0=stdin, 1=stdout, 2=stderr)
                            printf("TheOne-main: Client socket is valid (%d), preparing for file reception\n", client_fd);
                            // Clear receive buffer to ensure no command data is read
                            memset(mySocket.bufRecv, 0, sizeof(mySocket.bufRecv));
                            
                            // Start file reception 
                            printf("TheOne-main: Now starting file reception\n");
                            std::string received_filename;
                            if (FileRecvFromApp(&mySocket, "../Ver/", received_filename)) {
                                // File received successfully, check if it's a zip package
                                size_t dot_pos = received_filename.rfind('.');
                                if (dot_pos != std::string::npos && received_filename.substr(dot_pos) == ".zip") {
                                    std::string full_path = "../Ver/" + received_filename;
                                    myAll.CheckZipIntegrity(full_path);
                                }
                            }
                        } else {
                            printf("TheOne-main: Warning - Client socket is invalid (%d), cannot receive file\n", client_fd);
                            printf("TheOne-main: Cannot send error message to invalid socket\n");
                        }
                    } else if ( mySocket.bufRecv[1] == 'd' ) {
                        printf("TheOne-main: received from Android: [%s] (length=%d)\n", mySocket.bufRecv, recv_size);
                        printf("TheOne-main: Delete Map command\n");
                        myAll.MapDelete(mySocket.bufRecv + 3);
                    } else if ( mySocket.bufRecv[1] == 'f' ) {
                        printf("TheOne-main: received from Android: [%s] (length=%d)\n", mySocket.bufRecv, recv_size);
                        printf("TheOne-main: Send Map Content command\n");
                        mySocket.SendOneMapFile (mySocket.bufRecv + 4);
                    } else if ( mySocket.bufRecv[1] == 'l' ) {
                        printf("TheOne-main: received from Android: [%s] (length=%d)\n", mySocket.bufRecv, recv_size);
                        printf("TheOne-main: Send Map Name command\n");
                        //mySocket.SendAllMapFiles(myAll.get_hostapd_ssid().c_str());
                        char mapNameMsg[] = "$MapName\n";
                        mySocket.SendData(mapNameMsg);
                        std::string mapListJson = myAll.MapGetList();
                        mySocket.SendData((char*)mapListJson.c_str());
                    } else if ( mySocket.bufRecv[1] == 'M' ) { // receive map content
                        printf("TheOne-main: received from Android: [%s] (length=%d)\n", mySocket.bufRecv, recv_size);
                        myAll.MapRecvLine(mySocket.bufRecv, "../Map/");
                    } else {
                        printf("TheOne-main: received from Android: [%s] (length=%d)\n", mySocket.bufRecv, recv_size);
                        printf("TheOne-main socket recievd error: [%s]\n", mySocket.bufRecv); 
                    } 
                }
            }
        }
        timeCalculate(testtype, &time1, &timenow, "scoket recive");
    }
    
    //std::cout << "hello world" << std::endl;
    return 0;
}

// display the time cost of each part
int timeCalculate(int type, uint64_t *time1, uint64_t *timenow, const char *str) {
    static int seg_count=0;
    long long timecost=0;
    *timenow = T1Utils::getCurrentTimeUs();
    timecost = *timenow - *time1;
    if ( str[0] == '0' ) {
        seg_count = 0;
        if ( type == 2 )
            printf("Loop time cost:%8.3fms, usleep:%8.3fms\n", timecost/1000.0, 10 - (timecost/1000.0));
    } else {
        if ( type == 2 )
            printf("TheOne-main, time cost:%8.3fms,   segment %d: %s\n", timecost/1000.0, seg_count, str);
    }
    *time1 = *timenow;
    seg_count++;
    return (int)(timecost/1000);
}

// Transform the key or socket cmd to the suit format for serial port to F415
std::string tranCmdStr(T1All *pmyAll, char type, int key)
{
    char tmpstr[200]="\0";
    
//printf("TheOne-tranCmdStr('%c', [%c]), begin.\n", type, key);
    switch (key) {
    case '0':
        pmyAll->SetStatus('2',-1);  // stop Robot workAll
        pmyAll->SetStatus('R',-1);  // stop real Robot
        pmyAll->SetStatus('5',-1);  // stop testing Robot
        pmyAll->SetStatus('7',-1);  // stop All simulation, this line must be the last one!!!!!!
        pmyAll->SetStatus('6',-1);  // stop simulation, this line must be the last one!!!!!!
        pmyAll->SetStatus('3',-1);
        //myAll.SetStatus('T',-1);  // stop Trail Map
        break;
    case '1':// Log map by trail RTK point
    case 't':
        if ( pmyAll->GetSignal('T') < 0 ) {  // begin to log trail
            pmyAll->SetStatus('T',0);
        } else {  // stop log map trail
            pmyAll->SetStatus('T',-1);
        }
        break;
    case '2':
        //if ( pmyAll->GetSignal('2') < 0 ) {  // begin Robot All work
            //pmyAll->SetStatus('2',0);
        //} else {  // stop Robot workAll
            //pmyAll->SetStatus('2',-1);
        //} 
        break;
    case '3':
    case 'r':
        if ( pmyAll->GetSignal('3') < 0 ) {  // begin Robot All work, with Astar
            pmyAll->SetStatus('3',0);
        } else {
            if ( pmyAll->GetSignal('=') == 1 ) { // continue Robot All from pause
                pmyAll->SetStatus('=',-1);
            } else {   // stop Robot workAll
                pmyAll->SetStatus('3',-1);
            }
        } 
        break;
    case '4':
        if ( pmyAll->GetSignal('R') < 0 ) {  // begin Robot real work
            pmyAll->SetStatus('R',0);
        } else {  // stop Robot real work
            pmyAll->SetStatus('R',-1);
        }
        break;
    /*case '5':
        if ( pmyAll->GetSignal('5') < 0 ) {  // SMD real work
            pmyAll->SetStatus('5',0);  // testing real work
        } else {  // stop SMD work
            pmyAll->SetStatus('5',-1);
        } 
        break;*/    
    case '6':
        if ( pmyAll->GetSignal('6') < 0 ) {  // begin simulation on screen
            pmyAll->SetStatus('6',0);
        } else {  // stop simulation
            pmyAll->SetStatus('6',-1);
        } 
        break;
    case '7':
        if ( pmyAll->GetSignal('7') < 0 ) {  // begin All simulation on screen
            pmyAll->SetStatus('7',0);
        } else {
            if ( pmyAll->GetSignal('=') == 1 ) { // continue All simulation from pause
                pmyAll->SetStatus('=',-1);
            } else {  // stop All simulation
                pmyAll->SetStatus('7',-1);
            }
        }
        break;
    case 'h':
        if ( pmyAll->GetSignal('h') == 0 ) {  // begin go home
            pmyAll->SetStatus('h', 1);
        } else {  // stop go home, this may be never done ???
            pmyAll->SetStatus('h', 0);
        }
        break;
    case 'p':
        pmyAll->SetStatus('=',-1);  // pause/restart simulation and real work , -1 means reverse
        break;
    case 'c':
    case 'z':
        sprintf(tmpstr,"@CMD,%c=10\r\n", key);
        break;
    case '8':// walk forward for test
        printf("T1Main-tranCmdStr 8 pressed.\n");
        sprintf(tmpstr,"@CMD,w=100\r\n");
        break;
    case '9':// convert map by trail RTK points' Log
        //printf("T1Main-tranCmdStr 9 pressed.\n");
        pmyAll->SetStatus('9',1);  
        break;
    case 'f':
        printf("T1Main-tranCmdStr f pressed, walk forward for test.\n");
        sprintf(tmpstr,"@Cmd,w=100\r\n");
        break;
    case 'j':
        printf("T1Main-tranCmdStr j pressed.\n");
        pmyAll->SetStatus('S',5);  // fake vision 50
        break;
    case 'k':
        pmyAll->SetStatus('S',9);  // fake vision 60
        break;
    case 'i':
        //strcpy( tmpstr, readCmdStr().c_str() );
        sprintf(tmpstr,"@CMD,%s\r\n", readCmdStr().c_str() );
        //printf("T1Main-tranCmdStr, [%s]\n", tmpstr);
        break;
    case '<':
        pmyAll->MapSelectDemo(0);
        break;
    case '>':
        pmyAll->MapSelectDemo(1);
        break;
    case '-':
        pmyAll->MapDeleteDemo();
        break;
    case 'b':
        break;
    case 'u':
        pmyAll->CheckVersion("../t1Update", (const char* const[]){"t1Update", nullptr}, false);
        break;
    default:
        sprintf(tmpstr,"@CMD,%c\r\n", key);
        break;
    }
    
//printf("TheOne-tranCmdStr ('%c', [%c]), end.\n", type, key);
    
    std::string ret(tmpstr);
    return ret;
}

// read a command line from file and send through serial port to F415
std::string readCmdStr()
{
    std::string line = "0";
    const std::string filename = "./upt/CmdStr.upt";
    
    std::ifstream filein(filename);
    if (!filein.is_open() )
        return line.c_str();
    std::cout << "TheOne-readCmdStr, read: " << filename << std::endl;

    std::getline(filein,line);
    printf("TheOne-readCmdStr, CMD: [%s]\n", line.c_str() );
    
    filein.close();
    return line;
}



bool FileRecvFromApp(T1Socket *pmySocket, const std::string& save_path, std::string& received_filename) {
    printf("receiveFileFromApp: Start receiving file...\n");
    int client_fd = pmySocket->GetSocket('c');
    if (client_fd <= 0) {
        printf("receiveFileFromApp: Invalid client socket\n");
        return false;
    }
    
    // 1. Receive filename length (4 bytes)
    uint32_t filename_len = 0;
    int bytes_received = 0;
    char len_buf[4] = {0};
    while (bytes_received < 4) {
        int recv_size = pmySocket->RecvData(client_fd,1);
        if (recv_size == -1) {
            printf("receiveFileFromApp: Failed to receive filename length (error)\n");
            return false;
        } else if (recv_size == 0) {
            usleep(1000);
            continue;
        }
        int copy_len = (bytes_received + recv_size) > 4 ? (4 - bytes_received) : recv_size;
        memcpy(len_buf + bytes_received, pmySocket->bufRecv, copy_len);
        bytes_received += copy_len;
    }
    // Parse filename length, convert from network byte order to host byte order
    memcpy(&filename_len, len_buf, 4);
    filename_len = ntohl(filename_len);
    printf("receiveFileFromApp: Filename length: %u\n", filename_len);
    
    // Validate filename length
    if (filename_len == 0 || filename_len > 1024) {
        printf("receiveFileFromApp: Invalid filename length %u (must be between 1 and 1024)\n", filename_len);
        return false;
    }
    
    // 2. Receive filename
    std::string filename;
    filename.reserve(filename_len);
    bytes_received = 0;
    while (bytes_received < filename_len) {
        int recv_size = pmySocket->RecvData(client_fd,1);
        if (recv_size == -1) {
            printf("receiveFileFromApp: Failed to receive filename\n");
            return false;
        } else if (recv_size == 0) {
            usleep(1000);
            continue;
        }
        filename.append(pmySocket->bufRecv, recv_size);
        bytes_received += recv_size;
    }
    filename.resize(filename_len);
    printf("receiveFileFromApp: Receiving file: %s\n", filename.c_str());
    
    // 3. Receive file size (8 bytes)
    uint64_t file_size = 0;
    bytes_received = 0;
    char file_size_buf[8] = {0};
    while (bytes_received < 8) {
        int recv_size = pmySocket->RecvData(client_fd,1);
        if (recv_size == -1) {
            printf("receiveFileFromApp: Failed to receive file size\n");
            return false;
        } else if (recv_size == 0) {
            usleep(1000);
            continue;
        }
        int copy_len = (bytes_received + recv_size) > 8 ? (8 - bytes_received) : recv_size;
        memcpy(file_size_buf + bytes_received, pmySocket->bufRecv, copy_len);
        bytes_received += copy_len;
    }
    memcpy(&file_size, file_size_buf, 8);
    file_size = ntohll(file_size);
    printf("receiveFileFromApp: File size: %lu bytes\n", (unsigned long)file_size);
    
    // Validate file size (limit to 100MB)
    if (file_size == 0 || file_size > 100 * 1024 * 1024) {
        printf("receiveFileFromApp: Invalid file size %lu (must be between 1 and 100MB)\n", (unsigned long)file_size);
        return false;
    }
    
    // 4. Receive file data
    std::ofstream out_file(save_path + filename, std::ios::out | std::ios::binary | std::ios::trunc);
    if (!out_file.is_open()) {
        printf("receiveFileFromApp: Failed to create file: %s%s\n", save_path.c_str(), filename.c_str());
        return false;
    }
    
    uint64_t total_received = 0;
    while (total_received < file_size) {
        int recv_size = pmySocket->RecvData(client_fd,1);
        if (recv_size == -1) {
            printf("receiveFileFromApp: Connection closed unexpectedly\n");
            out_file.close();
            return false;
        } else if (recv_size == 0) {
            usleep(1000);
            continue;
        }
        
        // Write data to file, handle remaining bytes
        int write_len = (total_received + recv_size) > file_size ? (file_size - total_received) : recv_size;
        out_file.write(pmySocket->bufRecv, write_len);
        total_received += write_len;
        
        // Show transfer progress
        float progress = (float)total_received / file_size * 100;
        printf("\rreceiveFileFromApp: Transmission progress: %.1f%%", progress);
        fflush(stdout);
    }
    
    printf("\nreceiveFileFromApp: File received successfully! Saved to: %s%s\n", save_path.c_str(), filename.c_str());
    
    // Verify file size (ensure integrity)
    out_file.close();
    try {
        #ifdef __cpp_lib_filesystem
        std::uintmax_t saved_size = std::filesystem::file_size(save_path + filename);
        if (saved_size != file_size) {
            printf("receiveFileFromApp: Warning: File size mismatch! Possible data loss.\n");
            return false;
        }
        #else
        // Fallback for older compilers
        struct stat st;
        if (stat((save_path + filename).c_str(), &st) == 0) {
            uintmax_t saved_size = st.st_size;
            if (saved_size != file_size) {
                printf("receiveFileFromApp: Warning: File size mismatch! Possible data loss.\n");
                return false;
            }
        } else {
            printf("receiveFileFromApp: Error checking file size\n");
        }
        #endif
    } catch (const std::exception& e) {
        printf("receiveFileFromApp: Error checking file size: %s\n", e.what());
    }

    // Return the received filename
    received_filename = filename;
    
    return true;
}

// Manually implement 64-bit network byte order to host byte order conversion (compatible with all Linux systems)
uint64_t ntohll(uint64_t net_val) {
    static const int endian = 1;
    if (*(const char*)&endian == 1) {
        uint32_t high = ntohl((uint32_t)(net_val >> 32));
        uint32_t low = ntohl((uint32_t)(net_val & 0xFFFFFFFF));
        return ((uint64_t)low << 32) | high;
    } else {
        return net_val;
    }
}
