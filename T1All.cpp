#include "T1All.hpp"
#include "T1Utils.hpp"
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <iostream>
#include <chrono>
#include <fstream>
#include <dirent.h>
#include <algorithm>

T1All::T1All() {
    printf("\n\n\n\n\n=========================================\n");
    printf("=========================================\n");
    printf("=========================================\n");
    printf("=========================================\n");
    printf("=========================================\n");
    printf("=========================================\n");
    printf("T1All-T1All begin......\n");

    // Get SSID and store to T1Utils::meSsid
    T1Utils::get_hostapd_ssid();
    printf("T1All-T1All Got SSID: %s\n", T1Utils::meSsid.c_str());

    GetParamHeart();
    GetParamVersion();
    
    //pmyAct = &myActSimu;
    pmyAct = &myActRoboAll;

    GetSetting("./upt/WhoIsRobo.upt");
    GetSettingFactory();
    //CheckEnviroment();  // run in the GetSettingFactory()
    // if has rtk module, myFirm will reset rtk if timeout by this FlagRtkReset
    myFirm.SetStatus( 'K', pmyAct->GetStatus('K') );
    
    myMap.Init();
    myMap2.Init();
    myActSimuAll.pMap = &myMap2;
    myActRoboAll.pMap = &myMap2;
    // set gridmaps and update parameters after initialization
    MapSwitchNew(myMap.MapSelectRead().c_str());
    //printf("T1All step1......\n");
    myActSimu.pMap = &myMap;
    myActRobo.pMap = &myMap;
    //printf("T1All step2......\n");
    myActRobo.pmyVision = &myVision;
    myActRoboAll.pmyVision = &myVision;
    myVision.SetAvoidThresh(myActRoboAll.GetAvoidThresh());
    myVision.SetAvoidFreq(myActRoboAll.GetAvoidFreq());
    myVision.SetAprilTagSize(pmyAct->GetAprilTagSize(1));

    //printf("T1All step3......\n");
    myMower.init("./upt/SANF01.upt");
    myMower.noShift = 1;
    myMower.IDdevice = 101;
    myMower.devtype = 101; // SANF01, device type

    myDraw.Init();

    //printf("T1All step4......\n");
    // start vision thread
    visThread = std::thread(&T1All::ThreadVision, this);
    std::cout << "[T1All-T1All] Vision thread started" << std::endl;
}

T1All::~T1All() {
    // stop vision thread
    running = false;
    if (visThread.joinable()) {
        visThread.join();
        std::cout << "[T1All-~T1All] Vision thread stopped 0" << std::endl;
    }
    std::cout << "[T1All-~T1All] Vision thread stopped 1" << std::endl;
}

/* demo get data from vison thread
 * 2026.05.12: created by Leo
 */
PointRobo myVisionData;
std::mutex dataMutex;
void T1All::ThreadVision() {
    int debug = 0;
    running = true;

    PointRobo localData;
    localData.fx = 0.0f;
    localData.fy = 0.0f;
    localData.yaw = 0;
    //std::cout << "[T1All-ThreadVision] Detecting begin...[" << running << "]..." << std::endl;

    while (running) {
        uint64_t loopStart = T1Utils::getCurrentTimeMs();

        // Simulate vision processing time
        //std::cout << "[T1All-ThreadVision] loop..." << std::endl;
        //std::this_thread::sleep_for(std::chrono::milliseconds(200));

        int ret = myVision.Detect();
        if ( ret >= 0 && ret <= 100 ) {
            localData = myVision.GetPoint('A');
        } else if ( ret == 101 ) {
            localData = myVision.GetPoint('B');
        } else {
            //std::cout << "[T1All-ThreadVision] No tag detected." << std::endl;
        // Update detection result
            localData.fx = 1;
            localData.fy = 2;
            localData.yaw = 3;
            localData.flagPosSetBy = '=';
        }

        // Write to shared data safely
        std::lock_guard<std::mutex> lock(dataMutex);
        myVisionData = localData;

        // Control loop cycle to 300ms
        uint64_t elapsed = T1Utils::getCurrentTimeMs() - loopStart;
        int64_t sleepTime = 300 - (int64_t)elapsed;
        if ( debug > 0 ) printf("T1All-ThreadVision elapsed=%lu ms, sleep=%ld ms\n", elapsed, sleepTime);
        if (elapsed < 300) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
        }

        //printf("T1All-ThreadVision [Sent x,y,yaw]=%.0f,%.0f,%d\n",myVisionData.fx,myVisionData.fy,myVisionData.yaw);
    }
}

/* to drive vision OAK work or set status
 * 2025.10.13: created by Leo
 * 2026.05.12: modified by Leo, add the thread data get
 */
PointRobo T1All::ProcessVision(char type) {
    PointRobo TransData;
    switch(type) {
    case 'D':
        {   
            std::lock_guard<std::mutex> lock(dataMutex);
            TransData = myVisionData;
            myVisionData.flagPosSetBy = '=';
        }
        if ( TransData.flagPosSetBy == 'A' ) {
            pmyAct->SetStatus('A', 4);
            pmyAct->SetPointRobo('A', TransData);
            //printf("T1All-ProcessVision [Sent x,y,yaw]=%.0f,%.0f,%d\n",TransData.fx,TransData.fy,TransData.yaw);
        } else if ( TransData.flagPosSetBy == 'B' ) {
            Wall wall = myVision.GetWall();
            pmyAct->SetStatus('S', 8);
            pmyAct->SetWall('B', wall);
            //printf("T1All-ProcessVision No valid data from vision thread.\n");
        } else {
            //printf("T1All-ProcessVision, unknown flagPosSetBy[%c].\n", TransData.flagPosSetBy);
        }
        break;
    case 'C': // clean all FIFO
    /*case 'F':
        myVision.SetStatus('J', 1);
        break;
    case 'A':
        myVision.SetStatus('A', 1);
        break;*/
    default:
        printf("T1All-ProcessVision, unknown type[%c].\n",type);
        break;
    }
    return TransData;
}

/** receive and parse serial data line by line before callbqack recvLon()
 * 2026.05.16: merged two functions into one by Leo
 * 2026.06.02: add return flag, 
 *              >=0: success,
 *               -1: need reset 415
 *               -2: need send factory parameter
 */
int T1All::ProcessFirmSerial(int len, const unsigned char *data) {
    int ret = 0;
    MapPoint TransData;
    static int count = 0;
    static char line[1000];
    for (int i = 0; i < len; i++) {
        unsigned char c = data[i];
        if (c != '\n') { // accumulate characters until newline
            if ( count==999 ) count = 0;
            line[count] = c;
            count++;
            line[count] = '\0';
        } else {
            //if ( count==0) continue;
            if ( count==0) return 0;
            count = 0;
            char line1[1000];
            strcpy(line1, line);
            //printf("T1All-ProcessFirmSerial, line1=%s\n", line1);
            if ( strlen(line1)>0 && line1[0]=='$' ) {
                ret = myFirm.recvLon(line1);
                if ( ret<-2 ) {
                    //printf("T1All-ProcessFirmSerial, recvLon error for line: %s\n", line1);
                } else if ( ret==-2 ) {
                    printf("T1All-ProcessFirmSerial, recvLon return -2, send factory's parameter.\n");
                    //pmyAct->reset415();
                    ret=-2;
                } else if ( ret==-1 ) {
                    printf("T1All-ProcessFirmSerial, recvLon return -1, reset 415.\n");
                    //pmyAct->reset415();
                    ret=-1;
                } else if ( ret==0 ) { //$BNGGA/GPGGA success
                    //$command,GNGGA,,,,,,0,,,,,,,,,response: PARSING FAILD GRAMMAR ERROR,,,,,0,,,,,,,,*17
                    //printf("T1All-ProcessFirmSerial, recvLon $command GNGGA in-room RTK.\n");
                } else if ( ret==1 ) { //$BNGGA/GPGGA success
                    TransData = myFirm.GetPoint('R');
                    pmyAct->SetPointRTK('R', TransData);
                    pmyAct->ProcessGNGGA(line1);
                } else if ( ret==2 ) { //$roll success
                    TransData = myFirm.GetPoint('I');
                    pmyAct->SetPointRTK('I', TransData);
                    //printf("T1All-ProcessFirmSerial TransData roll,pitch,yaw=%d,%d,%d\n",TransData.roll,TransData.pitch,TransData.yaw);
                } else if ( ret==3 ) { //$stop success
                    TransData = myFirm.GetPoint('I');
                    pmyAct->SetPointRTK('I', TransData);
                    pmyAct->SetStatus('S', TransData.sate);
                } else if ( ret==4 ) { //$STA success - sync STA status to T1Act
                    pmyAct->UpdateFromSTA(myFirm.GetSTAStatus());
                } else if ( ret==5 ) { //$IMU success
                    TransData=myFirm.GetPoint('I');
                    pmyAct->SetPointRTK('I', TransData);
                } else {
                    printf("T1All-ProcessFirmSerial, ret=%d, line:[%s]\n", ret, line1);
                }
            }
        }
    }
    return ret;
}

/* get heartbeat parameters when system power up
 * 2026.02.14: created by Leo
 */ 
void T1All::GetParamHeart() {
    const char fileChecksum[] = "../Map/MapChecksum.dat";
    
    std::ifstream filein(fileChecksum, std::ios::in);
    if (!filein.is_open()) {
        T1Utils::LogBug("T1All-GetHeartParam, Can not open ../Map/MapChecksum.dat");
        return;
    }
    if ( !( filein >> meMapChecksum ) ) {
        T1Utils::LogBug("T1All-GetHeartParam, Can not read checksum from ../Map/MapChecksum.dat");
    } else {
        printf("T1All-GetHeartParam, checksum read=%lld\n", meMapChecksum);
    }
    filein.close();
}

/* get X3 hardware's version parameters when system power up
 * 2026.02.19: created by Leo
 */ 
void T1All::GetParamVersion() {
    const char fileVersion[] = "../Ver/Version";
    
    std::ifstream filein(fileVersion, std::ios::in);
    if (!filein.is_open()) {
        T1Utils::LogBug("T1All-GetVersionParam, Can not open ../Ver/Version");
        return;
    }
    if ( !( filein >> meHardwareVersionX3 ) ) {
        T1Utils::LogBug("T1All-GetVersionParam, Can not read checksum from ../Ver/Version");
    } else {
        printf("T1All-GetVersionParam, version HardwareX3 read=[%s]\n", meHardwareVersionX3);
    }
    filein.close();
}

void T1All::Draw(void)
{
    struct timeval tv;
    struct timezone tz;
    struct tm *t;
    char str[200];
//printf("T1All-Draw step 0.\n");    
    if ( pmyAct->GetStatus('R') >= 0 ) { // real work
        MapPoint tmpPoint = pmyAct->GetPoint('M',0);
        //printf("T1All-Draw(), real work, myRoboM:(%lld, %lld, %d)\n", tmpPoint.lon, tmpPoint.lat, tmpPoint.yaw);
        myMower.MoveSet(tmpPoint.lon, tmpPoint.lat, 0);
        myMower.MoveRotate(tmpPoint.yaw, 0);
    } else if ( pmyAct->GetStatus('5') >= 0 ) { // testing real walk
        MapPoint tmpPoint = pmyAct->GetPoint('M',0);
        //printf("T1All-Draw(), Testing:(%lld, %lld, %d)\n", tmpPoint.lon, tmpPoint.lat, tmpPoint.yaw);
        myMower.MoveSet(tmpPoint.lon, tmpPoint.lat, 0);
        myMower.MoveRotate(tmpPoint.yaw, 0);
    } else if ( pmyAct->GetStatus('3') >= 0 ) { // Robot WorkAll with Astar
        MapPoint tmpPoint = pmyAct->GetPoint('M',0);
        //printf("T1All-Draw(), Testing:(%lld, %lld, %d)\n", tmpPoint.lon, tmpPoint.lat, tmpPoint.yaw);
        myMower.MoveSet(tmpPoint.lon, tmpPoint.lat, 0);
        myMower.MoveRotate(tmpPoint.yaw, 0);
     } else if ( pmyAct->GetStatus('7') >= 0 ) { // All simulate on screen
        MapPoint tmpPoint = pmyAct->GetPoint('6',0);
        //printf("T1All-Draw(), All simulation:(%lld, %lld, %d)\n", tmpPoint.lon, tmpPoint.lat, tmpPoint.yaw);
        myMower.MoveSet(tmpPoint.lon, tmpPoint.lat, 0);
        myMower.MoveRotate(tmpPoint.yaw, 0);
    } else if ( pmyAct->GetStatus('6') >= 0 ) { // simulate on screen
        MapPoint tmpPoint = pmyAct->GetPoint('6',0);
        //printf("T1All-Draw(), simulation:(%lld, %lld, %d)\n", tmpPoint.lon, tmpPoint.lat, tmpPoint.yaw);
        myMower.MoveSet(tmpPoint.lon, tmpPoint.lat, 0);
        myMower.MoveRotate(tmpPoint.yaw, 0);
    } else {
        workDemoMove();
    }
//printf("T1All-Draw step 1.\n");    
    
    gettimeofday(&tv,&tz);
    t = localtime(&tv.tv_sec);
    sprintf(str,"%s %d-%d-%d %d:%d:%d.%ld", meTitleName, 1900+t->tm_year,1+t->tm_mon,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec,tv.tv_usec);
    //sprintf(str,"TheOne %d-%d-%d %d:%d:%d.%ld", 1900+t->tm_year,1+t->tm_mon,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec,tv.tv_usec);
    myDraw.DrawCommon(str, meHardwareVersionX3, pmyAct->GetHardwareVersionF415());
//printf("T1All-Draw step 2.\n");    
    //DrawLidar(painter,&myMowers[i]);
    
//printf("T1All-Draw step 3.\n");
    if ( pmyAct->GetStatus('T') >= 0 ) {
        myDraw.DrawMap( &myMap2 );
    } else if ( pmyAct->GetStatus('7') >= 0 ) {
        myDraw.DrawGridMap( &myMap2 );
        myDraw.DrawMap( &myMap2 );
        myDraw.DrawCutLine(((T1ActSimuAll *)pmyAct)->Cleft, ((T1ActSimuAll *)pmyAct)->Cright);
    } else if ( pmyAct->GetStatus('3') >= 0 ) {
        myDraw.DrawGridMap( &myMap2 );
        myDraw.DrawMap( &myMap2 );
    } else {
        myDraw.DrawMap( &myMap );
    }
    //myDraw.DrawMap( &myMap );
    myDraw.DrawActStatus(pmyAct);
    
    myDraw.DrawMower(&myMower);  

    //if ( meLamp == 1 ) DrawBang(&myBangs[i]);

    //drawer->DrawLidar(painter,IDLidarMower,fixLidar1,fixLidar2);
    //DrawAlarm(meAlarms,meID);
//printf("T1All-Draw step 4.\n");    
    myDraw.myFlush();
}

const char * T1All::GetString(char type)
{
    switch (type) {
    case 'S':
        return meSerialPort;
        break;
    case 'C':
        return pmyAct->StrCmdSend;
        break;
    case 'O':
        return pmyAct->StrSocketSend;
        break;
    case 'F':
        return pmyAct->StrFactory;
        break;
    default:
        return NULL;
        break;
    }
}

int T1All::GetSignal(char type)
{
    switch (type) {
    case 'K':
        return myDraw.SignalKey;
        break;
    case 'F':  // F415 key signal
        return pmyAct->Key415Signal;
        break;
    case 'T': // WorkStatusT
        return pmyAct->GetStatus('T');
        break;
    case 'S':
        return pmyAct->GetStatus('S');
        break;
    case 'X': //myRoboM.fx
        return pmyAct->GetStatus('X');
        break;
    case 'Y': //myRoboM.fy
        return pmyAct->GetStatus('Y');
        break;
    case 'y': //myRoboM.yaw
        return pmyAct->GetStatus('y');
        break;
    case 'P': // posFixedRP
        return pmyAct->GetStatus('P');
        break;
    case '=': // flagPause
        return pmyAct->GetStatus('=');
        break;
    case '7':
        return pmyAct->GetStatus('7');
        break;
    case '6':
        return pmyAct->GetStatus('6');
        break;
    case '5':
        return pmyAct->GetStatus('5');
        break;
    case 'R':
        return pmyAct->GetStatus('R');
        break;
    case '3':
        return pmyAct->GetStatus('3');
        break;
    case '2':
        return pmyAct->GetStatus('2');
        break;
    case 'N': // 1: test all the night,  2: display time cost
        return pmyAct->GetStatus('N');
        break;
    case 'h': // FlagGoHome
        return pmyAct->GetStatus('h');
        break;
    case 'i': //WorkStatusStaIn
        return pmyAct->GetStatus('i');
        break;
    case 'O': // WorkStatusStaOut
        return pmyAct->GetStatus('o');
        break;
    case 'b':
        return pmyAct->GetStatus('b');
        break;
    case 'V':
        return myVision.GetVisSignal('V');
        break;
    default:
        char strbug[200];
        sprintf(strbug,"T1All-GetSignal, unknown status type:[%c]",type);
        T1Utils::LogBug(strbug);
        return -1;
        break;
    }
}

//int T1All::GetVisualSignal(char type)
//{
    //switch (type) {
    //case 'V': // V: Avoid obstacle detection
        //return myVision.GetVisSignal('V');
        //break;
    //case 'B': // B: Boundary detection
        //return myVision.GetVisSignal('B');
        //break;
    //case 'A': // A: AprilTag detection
        //return myVision.GetVisSignal('A');
        //break;
    //case 'N': // N: All visual algorithms
        //return myVision.GetVisSignal('N');
        //break;
    //case 'R': // R: Whether to record video
        //return myVision.GetVisSignal('R');
        //break;
    //default:
        //char strbug[200];
        //sprintf(strbug,"T1All-GetVisualSignal, unknown status type:[%c]",type);
        //T1Utils::LogBug(strbug);
        //return -1;
        //break;
    //}
//}

void T1All::SetStatus(char type, int val)
{
    switch (type) {
    case 'K': // clean keyboard signal
        myDraw.SignalKey = val;
        break;
    case 'F': // clean F415 key signal
        pmyAct->Key415Signal = val;
        break;
    case 'S': // Log map by trail RTK's point
        pmyAct->SetStatus('S', val);
        break;
    case 'T': // Log map by trail RTK's point
        pmyAct = &myActRoboAll;  
        pmyAct->SetStatus('T', val);
        if ( val == -1 ) {
            //pmyAct->MapFromLog();   // act in T1Act
            MapSwitchNew(myMap.MapSelectRead().c_str());
            printf("T1All-SetStatus, T=%d, checksum=%lld\n", val, meMapChecksum);
        }
        break;
    case '7': // WorkAll simulate on screen
        pmyAct = &myActSimuAll;
        pmyAct->SetStatus('7', val);
        break;
    case '6': // simulate work on screen
        pmyAct = &myActSimu;
        pmyAct->SetStatus('6', val);
        break;
    /*case '5': // SMD real work
        pmyAct = &myActSMD;
        pmyAct->SetStatus('5', val);
        break;*/
    case '3': // robot begin to workAll with Astar
        if (val == 0) {  // Only reload when starting work (val==0), not when stopping (val==-1)
            //ReloadMapAndUpdateParams();
            printf("T1All-SetStatus('3'): Real robot work starting, reloaded maps. Cutter=%d, Grid=%d, Angle=%d\n",
                   myActSimuAll.GetStatus('c'), myActSimuAll.GetStatus('g'), myActSimuAll.GetStatus('a'));
        }
        pmyAct = &myActRoboAll;
        pmyAct->SetStatus('3', val);
        break;
    case '2': // robot begin to workAll, include goout/goin charge station
        //pmyAct = &myActRobo2;
        //pmyAct->SetStatus('2', val);
        break;
    case 'R': // robot begin to work, pressed key '4'
        pmyAct = &myActRobo;
        pmyAct->SetStatus('R', val);
        break;
    case '=': // pause simulation and real work, val==-1 means reverse
        pmyAct->SetStatus('=', val);
        break;
    case 'h': // return home, myActRoboAll/myActSimuAll
        //pmyAct = &myActRoboAll;
        pmyAct->SetStatus('h', val);
        break;
    /*case 'N': // 1: test all the night,  2: display time cost
        pmyAct->SetStatus('N', val);
        if ( val==4 ) myVision.SetStatus('S',1);
        break;*/
    case 'f': // resume traversal from saved context (return to point before going home)
        // Ensure resume always targets RoboAll state machine
        // Route through SetStatus so subclasses can handle it appropriately
        pmyAct = &myActRoboAll;
        pmyAct->SetStatus('f', val);
        break;
    case 'C': // reset CmdSend
        pmyAct->StrCmdSend[0] = 0;
        break;
    case 'O': // O not 0, reset SocketSend
        pmyAct->StrSocketSend[0] = 0;
        break;
    default:
        T1Utils::LogBug("T1All-SetStatus, unknown status type.");
        break;
    }
}

void T1All::LogTraversalStatus()
{
    static int statusOld = -1;
    int statusNew = pmyAct->GetStatus('3');

    if ( statusOld < 0 && statusNew >= 0 ) {
        T1Utils::LogRecord("traversal,start","Traversal",1);
    } else if ( statusOld >= 0 && statusNew < 0 ) {
        T1Utils::LogRecord("traversal,stop","Traversal",1);
    }

    statusOld = statusNew;
}

void T1All::Work()
{   
    
    //LowBatRecharge();

    //printf("T1All-RoboWork begin.\n");
    if ( pmyAct->GetStatus('T')==0 ) {
        ((T1ActRoboAll *)pmyAct)->WorkTrail();
    } else {
        pmyAct->WorkAll();
        char strVis[LENGTHSTRSEND] = "\0";
        pmyAct->getStrSend('V', strVis);
        if (strVis[0] != 0) {
            myVision.SetStatus('V', strVis[0]-'0'); // Convert char digit to int
            pmyAct->clearStrSend('V');
        }
    }

    LogTraversalStatus();
    //printf("T1All-RoboWork end.\n");
}

void T1All::workDemoMove(void)
{
    meCountTimer++;
    int sx,sy,sz,dx,dy,dz,mx,my,mz;
    dx = (meCountTimer%1000)/500;
    dy = (meCountTimer%2000)/1000;
    dz = (meCountTimer%720)/360;
    if ( dx<1 ) sx = meCountTimer%500;
    else sx = 499 - meCountTimer%500;
    if ( dy<1 ) sy = meCountTimer%1000;
    else sy = 999 - meCountTimer%1000;
    if ( dz<1 ) sz = meCountTimer%360;
    else sz = 359 - meCountTimer%360;
    mx = -500 + (sx-0)*2;
    my = 3000 - (sy-0)*7;
    mz = 0 + (sz-0)*3;
    if ( my < mx+10 ) my = mx+10;
    
    //printf("T1All-workDemoMove: %d,%d\n",mx,sz);
    myMower.MoveSet(0,mx,0);
    myMower.MoveRotate(sz,sz/4);
}

/** simulate the APP's command string
 * select map and set default map, 
 * 2026.02.20: created by Leo
 */ 
void T1All::MapSelectDemo(int sel) {
    //printf("\nT1All-MapSelectDemo begin, select=%d\n", sel);
    auto mapfiles = myMap.MapGetList();
    int mapMax = mapfiles.size();
    printf("\nT1All-MapSelectDemo map number=%d\n", mapMax);
    int count = 0;
    for (const auto& filename : mapfiles) {
        printf("T1All-MapSelectDemo loop:  %d. [%s]\n", count++, filename.c_str());
    }
    if ( sel == 0 ) {
        meMapCurrent--;
    } else if ( sel == 1 ) {
        meMapCurrent++;
    } else {
        printf("T1All-MapSelectDemo unknown sel=%d\n", sel);
    }
    if ( meMapCurrent < 0 ) meMapCurrent++;
    if ( meMapCurrent >= mapMax ) meMapCurrent--;
    printf("T1All-MapSelectDemo, select: %d.[%s]\n", meMapCurrent, mapfiles[meMapCurrent].c_str());

    MapSwitchNew(mapfiles[meMapCurrent].c_str());
}

/** Delete map and maybe need to reset default map, 
 * 2026.03.2: created by Leo
 */ 
void T1All::MapDeleteDemo() {
    std::string mapSelect = myMap.MapSelectRead();
    printf("T1All-MapDeleteDemo begin, select=[%s]\n", mapSelect.c_str());
    MapDelete(mapSelect.c_str());
}

/** Delete map and maybe need to reset default map, 
 * 2026.03.2: created by Leo
 */ 
void T1All::MapDelete(const char *mapfilename) {
    auto mapfiles = myMap.MapGetList();
    int mapMax = mapfiles.size();
    if ( mapMax <= 1 ) {
        printf("T1All-MapDelete step1, only 1 map left, can't del(%s)\n", mapfilename);
        return;
    }
    // delete the mapfile
    std::string mappath = "../Map/" + std::string(mapfilename);
    if (std::remove(mappath.c_str()) == 0) {
        std::cout << "T1All-MapDelete step2a, File deleted successfully. (" << mapfilename << ")\n";
    } else {
        // perror prints the specific system error (e.g., Permission Denied)
        perror("T1All-MapDelete step2b, Error deleting file");
        return;
    }
    // judge if select map be deleted
    printf("\nT1All-MapDelete step3, del(%s)\n", mapfilename);
    std::string mapSelect = myMap.MapSelectRead();
    if ( mapSelect == mapfilename ) { // must re-select another map,set to 0
        mapfiles = myMap.MapGetList();
        MapSwitchNew(mapfiles[0].c_str());
        printf("\nT1All-MapDelete step3a, reset(%s)\n", mapfiles[0].c_str());
    } else { // reset the checksum
        MapSwitchNew(mapSelect.c_str());
        printf("\nT1All-MapDelete step3a, reset(%s)\n", mapSelect.c_str());
    }
}

/** receive the selected mapfile's name from APP
 * select map and set default map, 
 * 2026.02.20: created by Leo
 */ 
void T1All::MapSwitchNew(const char *mapfilename) {
    printf("\nT1All-MapSwitchNew begin, select=[%s]\n", mapfilename);
    ResetGridParams(); // the gridmap's parameter my be changed if demo map used
    myMap.MapSelect(mapfilename);
    meMapChecksum = myMap.MapGetChecksum();
    
    // dispaly the select map on X11 windows
    char tmpstr[210];
    auto mapfiles = myMap.MapGetList();
    int mapMax = mapfiles.size();
    sprintf(tmpstr,"%d-%d %s", mapMax, meMapCurrent, mapfilename);
    myDraw.SetMapName(tmpstr);
    printf("T1All-MapSwitchNew end, SetMapName(%s)\n", tmpstr);
}

/** Process map receive line from App
 * Protocol: $M:StartMap -> $M:filename.map -> $M:content lines -> $M:EndMap
 * 2026.05.12: created by BY
 */ 
void T1All::MapRecvLine(const char* line, const std::string& save_path) {
    printf("MapRecvLine: Received: [%s]\n", line);
    
    // Check for start command $M:StartMap
    if (strncmp(line, "$M:StartMap", 11) == 0) {
        printf("MapRecvLine: StartMap command received\n");
        meMapReceiving = true;
        meMapFilenameReceived = false;
        meMapLineCount = 0;
        return;
    }
    
    // Check for end command $M:EndMap
    if (meMapReceiving && strncmp(line, "$M:EndMap", 9) == 0) {
        printf("MapRecvLine: EndMap command received\n");
        if (meMapOutFile.is_open()) {
            meMapOutFile.close();
            printf("MapRecvLine: Map file received successfully! Saved to: %s%s\n", save_path.c_str(), meMapFilename.c_str());
        }
        meMapReceiving = false;
        meMapFilenameReceived = false;
        return;
    }
    
    // First line after StartMap is the filename
    if (meMapReceiving && !meMapFilenameReceived) {
        meMapFilename = std::string(line + 3);
        size_t end_pos = meMapFilename.find("\r\n");
        if (end_pos != std::string::npos) {
            meMapFilename = meMapFilename.substr(0, end_pos);
        }
        printf("MapRecvLine: Map filename: %s\n", meMapFilename.c_str());
        
        meMapOutFile.open(save_path + meMapFilename, std::ios::out | std::ios::trunc);
        if (!meMapOutFile.is_open()) {
            printf("MapRecvLine: Failed to create file: %s%s\n", save_path.c_str(), meMapFilename.c_str());
            meMapReceiving = false;
            return;
        }
        meMapFilenameReceived = true;
        return;
    }
    
    // Receive map content lines and write to file
    if (meMapReceiving && meMapFilenameReceived && meMapOutFile.is_open()) {
        std::string line_content = std::string(line + 3);
        size_t end_pos = line_content.find("\r\n");
        if (end_pos != std::string::npos) {
            line_content = line_content.substr(0, end_pos);
        }
        meMapOutFile << line_content << std::endl;
        meMapLineCount++;
        printf("MapRecvLine: Receiving lines: %d\n", meMapLineCount);
        fflush(stdout);
    }
}

void T1All::MapSwitch(const char *StrApp) {
    char mapCmd = StrApp[0];
    char mapFile[256];

    bool needCopyToDefault = false; 
    if (mapCmd == '2') {
        sprintf(mapFile, "./upt/TrailBorder200x200.map");
        printf("T1main: Received M1, loading map: %s\n", mapFile);
        needCopyToDefault = true;
    }else if (mapCmd == '3') {
        sprintf(mapFile, "./upt/TrailBorder300x300.map");
        printf("T1main: Received M2, loading map: %s\n", mapFile);
        needCopyToDefault = true;
    }else if (mapCmd == 'R') {
        sprintf(mapFile, "../Map/TrailBorder.map");
        printf("T1main: Received MR, loading current map: %s\n", mapFile);
        needCopyToDefault = true;  
    }else if (StrApp != nullptr && strlen(StrApp) > 0) {
        sprintf(mapFile, "../Map/%s", StrApp);
        printf("T1main: Received map name: %s, loading map: %s\n", StrApp, mapFile);
        needCopyToDefault = true;
    }else {
        sprintf(mapFile, "./upt/TrailBorder.map");
        printf("T1main: Unknown map M%c, use default: %s\n", mapCmd, mapFile);
        needCopyToDefault = false;
    }
    if (needCopyToDefault) {
        if (!T1Utils::copyFile(mapFile, "../Map/MapDefault.maap")) {
            printf("T1main: Failed to set default map\n");
        }
    }
    //myActRoboAll.pMap->GetRoute(0);
    ResetGridParams();
}

/** reset all gridmap's parameters to factory setting before GetRemote()
 * 2026.02.23: created by Leo
 */
void T1All::ResetGridParams()
{
    printf("T1All::ResetGridParams, begin.\n");
    myMap2.SetValue('c', myActSimuAll.GetStatus('c'));
    myMap2.SetValue('g', myActSimuAll.GetStatus('g'));
    myMap2.SetValue('a', myActSimuAll.GetStatus('a'));
}

/* find the upt files begin with "Factory"
 * 2026.06.18: created by Leo
 */
void T1All::GetSettingFactory()
{
    printf("T1All-GetSettingFactory begin.\n");

    // check the ./upt/Factory*.upt files
    DIR *dir;
    struct dirent *entry;
    char factoryname[256];
    if ((dir = opendir("./upt")) != NULL) {
        while ((entry = readdir(dir)) != NULL) {
            // Check if the file has .upt extension
            const char *ext = strrchr(entry->d_name, '.');
            char begin[10];
            strncpy(begin,entry->d_name,7);
            if ( ext!=NULL && strcmp(ext, ".upt")==0 && strcmp(begin,"Factory")==0 ) {
                // Construct full file path
                //snprintf(filepath, sizeof(entry->d_name)-4, "%s", entry->d_name);
                strncpy(factoryname, entry->d_name, strlen(entry->d_name)-4);
                printf("T1All::GetSettingFactory, factory file: %s,%s,%ld\n", factoryname, entry->d_name,strlen(entry->d_name)-4);
                break;
            }
        }
        closedir(dir);
    } else {
        printf("T1All::GetSettingFactory, Failed to open directory: ./upt\n");
    }
    strncpy(meTitleName,factoryname,sizeof(meTitleName));
    CheckEnviroment(meTitleName);

    myActSimu.GetFactory( meTitleName );
    myActSimuAll.GetFactory( meTitleName );
    myActRobo.GetFactory( meTitleName );
    myActRoboAll.GetFactory( meTitleName );
}

void T1All::GetSetting(const char * name)
{
    char filename[100];
    sprintf(filename,"%s", name);
    std::ifstream filein(filename);
    if (!filein.is_open() )
        return;
    std::cout << "T1All-GetSetting, read: " << filename << std::endl;

    std::string line;
    std::vector<std::string> list1;
    //int tmptype = 0;

    int count = 0;
    while ( std::getline(filein,line) ) {
        count ++;
        list1 = T1Utils::split(line,' ');
        //std::cout << "reading: " << list1.size() << "[" << line << "]" << std::endl;
        if ( list1.size() < 2 ) continue;
        if ( line.substr(0,2) == "//" ) continue;
        if ( line.substr(0,5) == "#name" ) {
             sprintf(meTitleName,"%s", list1[1].c_str() );
             continue;
        }
        if ( line.substr(0,5) == "#port" ) {
             sprintf(meSerialPort,"%s", list1[1].c_str() );
             continue;
        }
        /*if ( line.substr(0,10) == "#testnight" ) { // 1: test all the night
            SetStatus('N', std::stoi(list1[1]));
            myActSimuAll.SetStatus('N', std::stoi(list1[1]));
            myActRoboAll.SetStatus('N', std::stoi(list1[1]));
            printf("T1All-GetSetting, testnight: %d\n", pmyAct->GetStatus('N'));
            continue;
        }*/
        if ( line.substr(0,7) == "#window" ) {
            if ( list1.size() != 2 ) continue;
            std::vector<std::string> sizes = T1Utils::split(list1[1],',');
            int winw = std::stoi(sizes[0]);
            int winh = std::stoi(sizes[1]);
            int lawnw = std::stoi(sizes[2]);
            int lawnh = std::stoi(sizes[3]);
            myDraw.SetWinSize(winw,winh,lawnw,lawnh);
            printf("T1All-GetSetting, SetWinSize: %d,%d,%d,%d\n", winw, winh, lawnw, lawnh);
            continue;
        }
        if ( line.substr(0,6) == "#shift" ) {
            if ( list1.size() != 2 ) continue;
            std::vector<std::string> shifts = T1Utils::split(list1[1],',');
            int shx = std::stoi(shifts[0]);
            int shy = std::stoi(shifts[1]);
            myDraw.SetShift(shx,shy);
            //printf("T1All-GetSetting, SetShift: %d,%d\n",shx,shy);
            continue;
        }
        if ( line.substr(0,6) == "#scale" ) {
            if ( list1.size() != 2 ) continue;
            std::vector<std::string> scales = T1Utils::split(list1[1],',');
            float sx = std::stof(scales[0]);
            float sy = std::stof(scales[1]);
            float sz = std::stof(scales[2]);
            myDraw.SetScale(sx,sy,sz);
            //printf("T1All-GetSetting, SetScale: %.3f,%.3f,%.3f\n",sx,sy,sz);
            continue;
        }
        if ( line.substr(0,6) == "#angle" ) {
            if ( list1.size() != 2 ) continue;
            std::vector<std::string> angles = T1Utils::split(list1[1],',');
            float ax = std::stof(angles[0]);
            float ay = std::stof(angles[1]);
            float az = std::stof(angles[2]);
            myDraw.SetRotate(ax,ay,az);
            //printf("T1All-GetSetting, angle SetRotate: %.3f,%.3f,%.3f\n",ax,ay,az);
            continue;
        }
        if ( line.substr(0,12) == "#borderwidth" ) {
            if ( list1.size() != 2 ) continue;
            int borderwidth = std::stoi(list1[1]);
            myMap2.SetValue('b',borderwidth);
            //printf("T1All-GetSetting, borderwidth: %d\n", borderwidth);
        }
        if ( line.substr(0,8) == "#logtype" ) {
            if ( list1.size() != 2 ) continue;
            myFirm.SetStatus('L',std::stoi(list1[1]));
            //printf("T1All-GetSetting, LogType: %d (0=off, 1=LogLonBug, 2=LogBug, 3=both)\n", meLogType);
            continue;
        }
        if ( line.substr(0,8) == "#karlman" ) {
            if ( list1.size() != 4 ) continue;
            int karlmanR = std::stoi(list1[1]);//RTK Karlman gain time threshold
            int karlmanA = std::stoi(list1[2]);//Visual AprialTag Karlman gain time threshold
            int karlmanS = std::stoi(list1[3]);//Speed adjust Karlman gain time threshold

            myActSimu.SetStatus('r', karlmanR);
            myActSimuAll.SetStatus('r', karlmanR);
            myActRobo.SetStatus('r', karlmanR);
            myActRoboAll.SetStatus('r', karlmanR);
            
            myActSimu.SetStatus('a', karlmanA);
            myActSimuAll.SetStatus('a', karlmanA);
            myActRobo.SetStatus('a', karlmanA);
            myActRoboAll.SetStatus('a', karlmanA);
            
            myActSimu.SetStatus('s', karlmanS);
            myActSimuAll.SetStatus('s', karlmanS);
            myActRobo.SetStatus('s', karlmanS);
            myActRoboAll.SetStatus('s', karlmanS);
            
            continue;
            continue;
        }
        if ( line.substr(0,8) == "#timeout" ) {
            if ( list1.size() != 2 ) continue;
            // pass logtype to all T1Act subclasses
            myFirm.SetStatus('T', std::stoi(list1[1])); 
            //printf("T1All-GetSetting, timeout: %d\n", std::stoi(list1[1]));
            continue;
        }
    }
    
    std::cout << "T1Mower-GetSetting: reading upt end." << count << std::endl;
    filein.close();

    //SetPBangs();
}

/*
 * JudgeWorkStatus - Determine robot current working state via signal detection
 * Return: 0=idle, 1=out of charge, 2=returning home, 3=working, 4=pausing while working
 * Signal '3': working signal, 'i': returning home signal, 'O': out of charge signal, '=': pausing signal
 * 2026.07.01: created by BY
 */
int T1All::JudgeWorkStatus(){

    int IsWorking = GetSignal('3')!=-1;
    int ReHome = GetSignal('i')!=-1;
    int OutCharge = GetSignal('O')!=-1;
    int IsPausing = GetSignal('=')==1;
    if(IsPausing&&IsWorking){
        return 4;
    }else if(OutCharge){
        return 1;
    }else if(ReHome){
        return 2;
    }else if(IsWorking){
        return 3;
    }else{
        return 0;
    }       
}

/*
 * LowBatRecharge - Trigger return charge when battery < 20% and robot is not already heading home/in station
 * SetStatus('h', 1) on trigger
 * 2026.07.01: created by BY
 */
void T1All::LowBatRecharge() {
    int batteryLevel = GetSignal('b');
    int workStatus = GetSignal('3');
    int isInStation = GetSignal('i');
    int isLeavingStation = GetSignal('O');
    bool isGoingHome = (workStatus == 71 || workStatus == 3 || workStatus == 4 || workStatus == 5 || workStatus == -1 || workStatus == 0 || workStatus == 1);
    if (batteryLevel > 0 && batteryLevel < 20 && !isGoingHome && isInStation == -1 && isLeavingStation == -1) {
        SetStatus('h', 1);
    }
}

/* 2026.02.13: moved from TheOne.cpp by Leo
 * 2026.03.06: edit by BY*/
int T1All::SetSysTime(const char* buf, int buf_len) {
    // 1. Print function entry information
    //printf("T1All:SetTime() Function started | Input buf address: %p | buf length: %d\n", (void*)buf, buf_len);
    //printf("T1All:SetTime() Function started | buf:[%s] | length=%d\n", buf, buf_len);

    // 2. Print before format validation
    //printf("T1All:SetTime() Starting input format validation...\n");
    if (buf == NULL || buf_len < 17 || buf[0] != '$' || buf[1] != 'T' || buf[2] != ':') {
        printf("T1All:SetSysTime() Invalid format | Length=%d\n", buf_len);
        return -1;
    }
    //printf("TheOne:SetTime() Input format validation passed\n");

    // 3. Print before parsing time fields
    //printf("T1All:SetTime() Starting to parse time fields...\n");
    int year = (buf[3]-'0')*1000 + (buf[4]-'0')*100 + (buf[5]-'0')*10 + (buf[6]-'0');
    int month = (buf[7]-'0')*10 + (buf[8]-'0');
    int day = (buf[9]-'0')*10 + (buf[10]-'0');
    int hour = (buf[11]-'0')*10 + (buf[12]-'0');
    int min = (buf[13]-'0')*10 + (buf[14]-'0');
    int sec = (buf[15]-'0')*10 + (buf[16]-'0');
    // Print parsing result
    printf("T1All-SetSysTime() Time field parsing completed | Year:%d Month:%d Day:%d Hour:%d Min:%d Sec:%d\n", 
           year, month, day, hour, min, sec);

    // 4. Print before time value validation
    //printf("T1All:SetTime() Starting time value validation...\n");
    if (year < 2000 || year > 2100 || month < 1 || month > 12 || day < 1 || day > 31) {
        printf("T1All:SetSysTime() Invalid time values\n");
        return -2;
    }
    //printf("T1All:SetTime() Time value validation passed\n");

    // 5. Print before constructing tm structure
    //printf("T1All:SetTime() Starting to construct tm time structure...\n");
    struct tm tm_time = {0};
    tm_time.tm_year = year - 1900;  // tm_year is years since 1900
    tm_time.tm_mon = month - 1;     // tm_mon is 0-11 (January is 0)
    tm_time.tm_mday = day;
    tm_time.tm_hour = hour;
    tm_time.tm_min = min;
    tm_time.tm_sec = sec;
    //printf("T1All:SetTime() tm time structure constructed successfully\n");

    // 6. Print before setting signal and alarm
    //printf("T1All:SetTime() Setting up SIGALRM signal handler and alarm...\n");
    signal(SIGALRM, [](int) { printf("T1All:SetTime() mktime timeout\n"); });
    alarm(3);  // Set 3 seconds timeout for mktime
    //printf("T1All:SetTime() Alarm set to 3 seconds\n");

    // 7. Print before timestamp conversion
    //printf("T1All:SetTime() Converting tm structure to timestamp...\n");
    time_t timestamp = mktime(&tm_time);
    alarm(0);  // Cancel the alarm
    //printf("T1All:SetTime() Alarm cancelled after mktime call\n");
    if (timestamp == (time_t)-1) { 
        printf("T1All:SetSysTime() Timestamp conversion failed\n");
        return -2;
    }
    //printf("T1All:SetTime() Timestamp conversion successful | Timestamp: %ld\n", (long)timestamp);

    // 8. Print before setting system time
    //printf("T1All:SetTime() Preparing to set system real-time clock...\n");
    
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "sudo date -s '%d-%02d-%02d %02d:%02d:%02d'", year, month, day, hour, min, sec);
    int ret = system(cmd);
    
    if (ret != 0) {
        printf("T1All:SetSysTime() Failed to set system time\n");
        return -3;
    }
    //printf("T1All:SetTime() System real-time clock set successfully\n");
 
    printf("T1All-SetSysTime() Time synchronization successful: %d-%02d-%02d %02d:%02d:%02d\n", year, month, day, hour, min, sec);
    
    // 9. Print function exit information
    //printf("T1All:SetTime() Function executed successfully, returning 0\n");
    return 0;
}

/** 2026.02.13: moved from TheOne.cpp by Leo
 * */
std::string T1All::getHeartBeatStr() {
    int mapfixed = GetSignal('P');
    int myRobomX = GetSignal('X');
    int myRobomY = GetSignal('Y');
    int myRobomy = GetSignal('y');
    int statA = GetSignal('S');
    int status = JudgeWorkStatus();
    int batteryLevel = GetSignal('b');
    
    std::string ssid = "unknown_ssid";    
    std::string version = "0.0.0";        
    std::string currzip = "0.0.0";     

    ssid = T1Utils::meSsid;
 
    try {
        version = get_version();
    } catch (const std::runtime_error& e) {

    } catch (...) {
    }

    try {
        std::string temp_zip = getMaxZip();
        if (!temp_zip.empty()) {
            currzip = temp_zip;
        } else {
            throw std::runtime_error("empty zip version");
        }
    } catch (const std::runtime_error& e) {

    } catch (...) {
    
    }
    
    std::string ret = 
        "ssid:" + ssid + 
        "V:" + version +
        "Z:" + currzip + 
        "P:" + std::to_string(myRobomX) + "," + std::to_string(myRobomY) + "," + std::to_string(myRobomy) + 
        "F:" + std::to_string(mapfixed) + 
        "M:" + std::to_string(meMapChecksum) + 
        "N:" + std::to_string(statA) + 
        "S:" + std::to_string(status) +  
        "B:" + std::to_string(batteryLevel) + "\n";
    //printf("T1All-getHeartBeatStr end, ret=[%s]\n", ret.c_str());
    return ret;
}

/* 2026.02.13: moved from TheOne.cpp by Leo
 * */
std::string T1All::get_version() {//const std::string& version_path) {
    const std::string version_path = "../Ver/Version";
    if (access(version_path.c_str(), R_OK) != 0) {
        throw std::runtime_error("Failed to access Version file: " + version_path + " (insufficient permissions or file does not exist)");
    }

    std::ifstream file(version_path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open Version file: " + version_path);
    }

    std::string version;
    // Read entire line (ignore trailing newline)
    if (!std::getline(file, version)) {
        throw std::runtime_error("Version file is empty");
    }

    file.close();

    // Validate version format (optional: allow only digits and dots)
    const std::regex version_regex(R"(^[\d\.]+$)");
    if (!std::regex_match(version, version_regex)) {
        throw std::runtime_error("Invalid Version file format: " + version + " (only digits and dots are allowed)");
    }

    return version;
}

/* 2026.02.13: moved from TheOne.cpp by Leo
 * */
std::string T1All::getMaxZip() {
    std::string maxV = "";
    DIR* d = opendir("../Ver");
    if (!d) return "";

    dirent* e;
    while ((e = readdir(d))) {
        std::string f = e->d_name;
        struct stat st;
        if (stat(("../Ver/"+f).c_str(), &st) || !S_ISREG(st.st_mode) || 
            f.find("UpdateV")!=0 || f.rfind(".zip")!=f.length()-4) {
            continue;
        }
        std::string v = f.substr(7, f.length()-11);

        if (maxV.empty()) {
            maxV = v;
            continue;
        }

        auto split_version = [](const std::string& ver) -> std::vector<int> {
            std::vector<int> res;
            std::istringstream iss(ver);
            std::string part;
            while (getline(iss, part, '.')) {
                res.push_back(atoi(part.c_str()));
            }
            return res;
        };

        std::vector<int> max_ver = split_version(maxV);
        std::vector<int> cur_ver = split_version(v);
        bool is_current_bigger = false;

        int max_len = std::max(max_ver.size(), cur_ver.size());
        for (int i = 0; i < max_len; ++i) {
            int n1 = (i < max_ver.size()) ? max_ver[i] : 0;
            int n2 = (i < cur_ver.size()) ? cur_ver[i] : 0;
            
            if (n2 > n1) {
                is_current_bigger = true;
                break;
            } else if (n2 < n1) {
                break;
            }
        }

        if (is_current_bigger) {
            maxV = v;
        }
    }
    closedir(d);
    return maxV;
}

/* check if all directory exist, and copy necessary files to specified directory
 * 2026.02.10: creat by Leo
 * 2026.04.26: modified by Leo
 */
void T1All::CheckEnviroment(const char* factoryname) {
    printf("T1All-CheckEnviroment begin. %s\n", factoryname);

    // check ../Yuv, ../Map, ../Log, ../Ver,       if not exist, creat them
    const char* dirs[] = {"../Yuv", "../Map", "../Tsk", "../Log", "../Ver","../image","../Upt"};
    int num_dirs = sizeof(dirs) / sizeof(dirs[0]);
    for (int i = 0; i < num_dirs; i++) {
        struct stat st;
        if (stat(dirs[i], &st) == -1) {
            // Directory does not exist, create it
            if (mkdir(dirs[i], 0777) == 0) {
                printf("T1All::CheckEnviroment, Created directory: %s\n", dirs[i]);
                if ( dirs[i][3] == 'M' && dirs[i][4] == 'a' ) { // ../Map
                    //printf("T1All-CheckEnviroment copy map files.\n");
                    T1Utils::copyFile("./upt/TrailBorder100x100.map", "../Map/TrailBorder100x100.map");
                    T1Utils::copyFile("./upt/TrailBorder200x200.map", "../Map/TrailBorder200x200.map");
                    MapSwitchNew("TrailBorder200x200.map"); // set the selected map
                    printf("T1All-CheckEnviroment copy 2 map files.\n");
                }
                if ( dirs[i][3] == 'U' && dirs[i][4] == 'p' ) { // ../Upt
                    //printf("T1All-CheckEnviroment copy factory**single files.\n");
                    char filename1[200],filename2[200];
                    sprintf(filename1,"./upt/Single%s.upt", factoryname);
                    sprintf(filename2,"../Upt/Single%s.upt", factoryname);
                    T1Utils::copyFile(filename1,filename2);
                    printf("T1All-CheckEnviroment copy 1 SingleFactory**.upt files.\n");
                }
            } else {
                printf("T1All::CheckEnviroment, Failed to create directory: %s\n", dirs[i]);
            }
        } else if (S_ISDIR(st.st_mode)) {
            //printf("T1All::CheckEnviroment, Directory already exists: %s\n", dirs[i]);
        } else {
            printf("T1All::CheckEnviroment, Path exists but is not a directory: %s\n", dirs[i]);
        }
    }

    // delete ../Yuv/*.yuv
    DIR *dir;
    struct dirent *entry;
    char filepath[512];
    if ((dir = opendir("../Yuv")) != NULL) {
        while ((entry = readdir(dir)) != NULL) {
            // Check if the file has .yuv extension
            const char *ext = strrchr(entry->d_name, '.');
            if (ext != NULL && strcmp(ext, ".yuv") == 0) {
                // Construct full file path
                snprintf(filepath, sizeof(filepath), "../Yuv/%s", entry->d_name);
                // Delete the file
                if (remove(filepath) == 0) {
                    printf("T1All::CheckEnviroment, Deleted file: %s\n", filepath);
                } else {
                    printf("T1All::CheckEnviroment, Failed to delete file: %s\n", filepath);
                }
            }
        } 
        closedir(dir);
    } else {
        printf("T1All::CheckEnviroment, Failed to open directory: ../Yuv\n");
    }
}

// check the new software pack's version, and update
void T1All::CheckVersion(const char* target, const char* const args[], bool background) {
    // clear all memory
    ProcessVision('C');
    if (!target || strlen(target) ==0){
        std::cerr << "error target is null" << std::endl;
        exit(EXIT_FAILURE);
    }
    if (!args || args[0] == nullptr){
        std::cerr << "error args[] is wrong" << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << "start programe" << target << (background ? "run background":"run forward") << std::endl;
    if(background){
        char cmd_buf[1024] = {0};
        strncpy(cmd_buf,target,sizeof(cmd_buf) -1);
        for(int i = 1;args[i] != nullptr && strlen(cmd_buf) < sizeof(cmd_buf) - 2; i++){
            strncat(cmd_buf,"",1);
            strncat(cmd_buf,args[i],sizeof(cmd_buf) - strlen(cmd_buf) - 1);
        }
        strncat(cmd_buf," &",2);
        const char* const bash_args[] = {"bash","-c",cmd_buf,nullptr};
        int ret = execvp("bash",const_cast<char* const*>(bash_args));
        std::cerr << "run backgroud fail" << strerror(errno) << std::endl;
    }else{
        int ret = execvp(target,const_cast<char* const*>(args));
        std::cerr << "run forward fail" << strerror(errno) << std::endl;
    }

    exit(EXIT_FAILURE);
    // excl() to start T1Update

}

bool T1All::CheckZipIntegrity(const std::string& zip_path) {
    printf("TheOne-main: Detected zip package, checking integrity...\n");
    
    char test_cmd[512] = {0};
    snprintf(test_cmd, sizeof(test_cmd), "unzip -t \"%s\"", zip_path.c_str());
    
    int ret = system(test_cmd);
    if (WIFEXITED(ret) && WEXITSTATUS(ret) == 0) {
        printf("TheOne-main: Zip package integrity check passed\n");
        return true;
    } else {
        int exit_code = WEXITSTATUS(ret);
        printf("TheOne-main: Error: Zip package is corrupted or invalid! Return code: %d\n", exit_code);
        printf("TheOne-main: Deleting corrupted zip package: %s\n", zip_path.c_str());
         
        if (remove(zip_path.c_str()) == 0) {
            printf("TheOne-main: Corrupted zip package deleted successfully\n");
        } else {
            printf("TheOne-main: Warning: Failed to delete corrupted zip package\n");
        }
        return false;
    }
}

string T1All::MapGetList() {
    auto mapFiles = myMap.MapGetList();
    string json = "[";
    for (size_t i = 0; i < mapFiles.size(); i++) {
        json += mapFiles[i];
        if (i < mapFiles.size() - 1) {
            json += ",";
        }
    }
    json += "]";
    printf("T1All-MapGetList: JSON = %s\n", json.c_str()); 
    return json;
}

/** Get all traversal logs saved on device
 * 2026.08.03: created
 */
std::vector<std::string> T1All::GetLogList() {
    std::cout << "\nT1All-GetLogList, begin." << std::endl;

    char logFile[300];
    char bugMsg[400];
    std::string logPath = "../Log";
    std::vector<std::string> logFiles;
    std::vector<std::string> logList;

    DIR *dir = opendir(logPath.c_str());
    if ( dir == nullptr ) {
        sprintf(bugMsg,"T1All-GetLogList, Cannot open log path: %s",logPath.c_str());
        T1Utils::LogBug(bugMsg);
        return logList;
    }

    struct dirent *entry;
    while ( (entry = readdir(dir)) != nullptr ) {
        std::string filename = entry->d_name;
        if ( filename.size() >= 13 &&
             filename.substr(0,9) == "Traversal" &&
             filename.substr(filename.size()-4) == ".log" ) {
            logFiles.push_back(filename);
        }
    }
    closedir(dir);
    std::sort(logFiles.begin(),logFiles.end());

    for ( const auto& filename : logFiles ) {
        sprintf(logFile,"../Log/%s",filename.c_str());
        std::ifstream filein(logFile);
        if ( !filein.is_open() ) {
            sprintf(bugMsg,"T1All-GetLogList, Cannot open log file: %s",logFile);
            T1Utils::LogBug(bugMsg);
            continue;
        }

        std::string line;
        while ( std::getline(filein,line) ) {
            if ( line.empty() ) continue;
            logList.push_back("@Log," + line + "\n");
        }
        filein.close();
    }

    printf("T1All-GetLogList end, Length=%d\n",(int)logList.size());
    return logList;
}
