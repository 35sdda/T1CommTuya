#include "T1Act.hpp"
#include "T1Utils.hpp"
//#include <vector>
//#include <string>
#include <cstring>
#include <cstdint>
#include <iostream>
//#include <fstream>
#include <stdexcept>
#include <cstdio>
#include <math.h>
#include <ctime>
#include <dirent.h>

// int T1Act::FlagInCharge = 0;
// int T1Act::IsCharing = 0;

T1Act::T1Act() {
    printf("T1Act-T1Act() begin......\n");
    pmyVision = nullptr;
}

T1Act::~T1Act() {
    
}

/* Initializa the Robot's position before traversal
 */
void T1Act::RoboPosInitial() {
    //printf("T1ActRobotPosInitial begin: ");
    //if ( posFixedRP == 4 ) {
    //    ;
    //} else {    // in room, or no satellite signal
        myRoboM.fx = 0.0;//mapBase.lon;  // ??? r u sure?
        myRoboM.fy = 0.0;//mapBase.lat;  // ??? r u sure?
        myRoboM.yaw = myRoboI.yaw;   // ??? r u sure? the rtk will give 0 after powerup indoor
        myDiffMI.yaw = myRoboM.yaw - myRoboI.yaw;
    //}

    printf("T1Act-RobotPosInitial end: %d, %d, %d\n", myRoboM.yaw, myRoboI.yaw, myDiffMI.yaw);
}

/* Change the Robot's position after one step act
 * 2026.05.06: created by Leo
 */
void T1Act::RoboPosChange(char act, int val, int ret) {
    printf("T1Act-RoboPosChange begin, not support now: (%d,%d), cmd=[%c,%d,%d], myRoboM(%.1f,%.1f,%d), IMU:%d\n", meWorkline, meWorkstep, act, val, ret, myRoboM.fx, myRoboM.fy, myRoboM.yaw, myRoboI.yaw);
}

/* Change the Robot's position after one step act
 */
void T1Act::PosiSetByWheelO(char act, int val, int ret) {
    printf("T1Act-PosiSetByWheelO begin, not support now: (%d,%d), cmd=[%c,%d,%d], myRoboM(%.1f,%.1f,%d), IMU:%d\n", meWorkline, meWorkstep, act, val, ret, myRoboM.fx, myRoboM.fy, myRoboM.yaw, myRoboI.yaw);
}

/* The New Version for Robot Act in each time split, only get system time, no count split
 * return:  -1:not work,   0:working,    >0:time of working (ms, impossible large than 30 seconds)
 */
int T1Act::RoboAct(char act, int val) {
    printf("T1Act-RobotAct, not support now.\n");
    return -1;
}

/* In one line of the map travarsal, simulate the Mower goto the target point step by step
 * return==0, successfully finished the line work,   return>0: steps worked in this line
 * return<0: failed in this line, but not used now
 */
int T1Act::WorkLine(MapPoint end) {
    printf("T1Act-WorkLine begin: line=%d, step=%d, end:(%lld,%lld,%d)\n", meWorkline, meWorkstep, end.lon, end.lat, end.alt);
    return -1;
}
/* 
* return: 1= In execution, -1= completed, 0= paused
 */
int T1Act::ReturnHome() {
    printf("T1Act-ReturnHome() default implementation: not implemented in base class\n");
    printf("  Subclass should override this function to implement actual return home logic\n");
    return -1;
}

/* return==1: working,  return<0: finished work,
 * this include the action from ChargeStation and back to ChargeStation
 */
int T1Act::WorkAll() {
    //+printf("T1Act-WorkAll begin: (%d,%d,%d), RS6=%d, min(%lld,%lld), max(%lld,%lld)\n", meWorklength, meWorkline, meWorkstep, WorkStatusSimu, mapMin.lon, mapMin.lat, mapMax.lon, mapMax.lat);
    return Work();
    //return -1;
}

/* return==1: working,  return<0: finished work,
 */
int T1Act::Work() {
    printf("T1Act-Work begin: (%d,%d,%d), RS6=%d, min(%lld,%lld), max(%lld,%lld)\n", meWorklength, meWorkline, meWorkstep, WorkStatusSimu, mapMin.lon, mapMin.lat, mapMax.lon, mapMax.lat);
    return -1;
}

/* return==1: working,  return<0: finished work,
 * use the RoutePlan2
 */
int T1Act::Work2() {
    printf("T1Act-Work2 begin: (%d,%d,%d), RS6=%d, min(%lld,%lld), max(%lld,%lld)\n", meWorklength, meWorkline, meWorkstep, WorkStatusSimu, mapMin.lon, mapMin.lat, mapMax.lon, mapMax.lat);
    return -1;
}

int T1Act::CheckRoboActStatus() {
    return RoboAct('=', 0);
}

void T1Act::SetStatus(char name, int val) {
//printf("T1Act-SetStatus() begin. %c, %d\n", name, val);
    switch (name) {
    case 'N': // 1: test all the night;
        FlagTestNight = val;
        break;
    case 'R': // robot begin to work;
        WorkStatusR = val;
        //if ( val==0 ) RoboAct('0',0);
        break;
    /*case '2':
        break;*/
    case '3':
        WorkStatusRoboAll = val;
        if ( val == -1 ) {
            WorkStatusR = -1;
            WorkStatusStaOut = -1;
            WorkStatusStaIn = -1;
            WorkStatusPolar = 1;
            WorkStatusDetect = 0;
            IsCharing = 0;
            RoboAct('g',0);
            int WorkStatusGH = -1;
        }
        break;
    /*case '5':
        WorkStatus5 = val;
        break;*/
    case '6':
        WorkStatusSimu = val;
        break;
    case '7':
//printf("T1Act-SetStatus case: %c, %d\n", name, val);
        WorkStatusSimuAll = val;
        if ( val == -1 ) { // reset all child WorkStatus
            WorkStatusSimu = -1;
            WorkStatusStaOut = -1;
            WorkStatusStaIn = -1;
            WorkStatusGH = -1;
        }
        break;
    case '=': // pause simulation and real work
        if ( val == -1 ) { // reverse
            if ( FlagPause == 0 ){
                FlagPause = 1;
                RoboAct('g',0);
            } else {
                FlagPause = 0;
                }
        } else {
            FlagPause = val;
        }
        break;
    case 'h':
        printf("T1Act-SetStatus h, name=%c, val=%d\n", name, val);
        RoboAct('g',0);
        if ( val == -1 ) { // reverse
            if ( FlagGoHome == 0 ){
                FlagGoHome = 1;
                
            } else {
                FlagGoHome = 0;
            }
        } else {
            FlagGoHome = val;
        }
        break;
    case 'r': //RTK Karlman gain time threshold
        meKarlmanR = val;
        //printf("T1Act-SetStatus KarlmanR=%d\n", meKarlmanR);
        break;
    case 'a': //Visual AprialTag Karlman gain time threshold
        meKarlmanA = val;
        //printf("T1Act-SetStatus KarlmanA=%d\n", meKarlmanA);
        break;
    case 's': //Speed adjust Karlman gain time threshold
        meKarlmanS = val;
        //printf("T1Act-SetStatus KarlmanS=%d\n", meKarlmanS);
        break;
    case 'S':
        FlagStop = val;
        //printf("T1Act-SetStaus S=%d\n",val);
        break;
    // case 'I':
    //     FlagInfomation = (uint8_t)val;
    //     //FlagSpeed = FlagSpeedCount * 6;
    //     //FlagSpeedCount = val;
    //     break;
    case 'T': // trail map
        WorkStatusT = val;
        if ( WorkStatusT == -1 ) {
            printf("T1Act-SetStatus, T=%d\n", val);
            pMap->MapFromLog(FlagTestNight);
            //MapGetChecksum();  // act in T1All
        } else if ( WorkStatusT==0 ) { // station out first
            WorkStatusStaOut=0;
        }
        break;
    case 'f':
        printf("T1Act-SetStatus: F pressed, but no saved context found or resume not supported\n");
        break;
    case 'A':
        posFixedVA = val;
        //printf("T1Act-SetStatus: A pressed, val=%d\n", val);
        break;
    case 'B':
        posFixedVB = val;
        //printf("T1Act-SetStatus: B pressed, val=%d\n", val);
        break;
    default:
        printf("T1Act-SetStatus unknown error, name=%c, val=%d\n", name, val);
        break;
    }
//printf("T1Act-SetStatus() end. WS5=%d, WS6=%d, WSR=%d, WST=%d\n", WorkStatus5, WorkStatusSimu, WorkStatusR, WorkStatusT);
}

int T1Act::GetStatus(char name) {
    //std::cout << "T1Act-SetStatus begin." << name << val;
    switch (name) {
    case 'T': // log map by trail RTK's point
        return WorkStatusT;
        break;
    case '7': // WorkAll simulation route on screen
        return WorkStatusSimuAll;
        break;
    case '6': // simulation route on screen
        return WorkStatusSimu;
        break;
    /*case '5': // testing realwork
        return WorkStatus5;
        break;*/
    case 'K': // if has a RTK module
        return meFact.RtkSetFlag;
        break;
    case 'R': // real robot work
        return WorkStatusR;
        break;
    case 'S':
        return StataNum;
        break;
    case 'X':
        return (int)myRoboM.fx;
        break; 
    case 'Y':
        return (int)myRoboM.fy;
        break;
    case 'y':
        return (int)myRoboM.yaw;
        break;
    case '3': // real robot All work with Astar, include charge station in/out
        return WorkStatusRoboAll;
        break;
    case 'P':
        return posFixedRP;
        break;
    case '=':
        return FlagPause;
        break;
    case 'N':
        return FlagTestNight;
        break;
    case 'C':
        return FlagImuCount;
        break;
    case 'c':
        return meFact.StepLinec;
        break;
    case 'g':
        return meFact.StepLineg;
        break;
    case 'a':
        return meFact.StepLinea;
        break;
    case 'I':
        return FlagInfomation;
        break;
    case 'h':
        return FlagGoHome;
        break;
    case 'H':
        return WorkStatusGH;
        break;
    case 'i':
        return WorkStatusStaIn;
        break;
    case 'o':
        return WorkStatusStaOut;
        break;
    case 'b':
        return batteryLevel;
        break;
    case 'e':
        return meStageNorth;
        break;
    case 'n':
        return meAngleNorth;
        break;
    case 's':
        return meAngleStation;
        break;
    case '[':
        return meDiffStation;//meAngleNorth;
        break;
    case ']':
        return meDiffNorth;//meAngleStation;
        break;
    default:
        return -1;
        break;
    }
}

void T1Act::SetPointRobo(char name, PointRobo point) {
    switch (name) {
    case 'A':
        myRoboVA.fx = point.fx;
        myRoboVA.fy = point.fy;
        myRoboVA.yaw = point.yaw;
        myRoboVA.timeSet = point.timeSet;
        myRoboVA.flagPosSetBy = point.flagPosSetBy;
        //printf("T1Act-SetPoint A: (%.1f,%.1f,%d), timeSet=%ld, flagPosSetBy=%d\n", myRoboVA.fx, myRoboVA.fy, myRoboVA.yaw, myRoboVA.timeSet, myRoboVA.flagPosSetBy);
        break;
    default:
        printf("T1Act-SetPointRobo unknown error, name=%c\n", name);
        break;
    }
}

/*
 * SetWall - Set wall data from vision detection
 * Parameters:
 *   name: Target identifier ('B' for SGBM wall data)
 *   wall: Wall structure with left/right edge depth and horizontal offset
 * 2026.05.22: created by BY
 */
void T1Act::SetWall(char name, Wall wall) {
    switch (name) {
    case 'B':
        mySgbmWall.leftDepth = wall.leftDepth;
        mySgbmWall.leftOffset = wall.leftOffset;
        mySgbmWall.rightDepth = wall.rightDepth;
        mySgbmWall.rightOffset = wall.rightOffset;
        //printf("T1Act-SetWall B: leftDepth=%.1f, leftOffset=%.1f, rightDepth=%.1f, rightOffset=%.1f\n",
        //       mySgbmWall.leftDepth, mySgbmWall.leftOffset, mySgbmWall.rightDepth, mySgbmWall.rightOffset);
        break;
    default:
        printf("T1Act-SetWall unknown error, name=%c\n", name);
        break;
    }
}

void T1Act::SetPointRTK(char name, MapPoint point) {
    switch (name) {
    case 'I':
        myRoboI.roll = point.roll;
        myRoboI.pitch = point.pitch;
        myRoboI.yaw = point.yaw;
        myRoboI.gyroX=point.gyroX;
        myRoboI.gyroY=point.gyroY;
        myRoboI.gyroZ=point.gyroZ;
        myRoboI.accX=point.accX;
        myRoboI.accY=point.accY;
        myRoboI.accZ=point.accZ;
        myRoboI.timeIMU=point.timeIMU;
        myRoboI.timeIMU415=point.timeIMU415;
        myRoboI.timeSet = point.timeSet;
        FlagImuCount = point.alt;
        myRoboM.yaw = myRoboI.yaw + myDiffMI.yaw;
        while ( myRoboM.yaw>=360 ) myRoboM.yaw -= 360;
        while ( myRoboM.yaw<0 ) myRoboM.yaw += 360;
        //printf("T1Act-SetPointRTK I: (%.1f,%.1f,%d), timeSet=%ld, flagPosSetBy=%d\n", myRoboI.fx, myRoboI.fy, myRoboI.yaw, myRoboI.timeSet, myRoboI.flagPosSetBy);
        break;
    case 'R': // for stop, only update yaw and timeSet, use alt to flag count of stop
        myPosRTK.lon = point.lon;
        myPosRTK.lat = point.lat;
        myPosRTK.fixed = point.fixed;
        myPosRTK.sate = point.sate;
        myPosRTK.timeSet = point.timeSet;

        //printf("T1Act-SetPointRTK R: (%lld,%.lld,%d,%d), timeSet=%ld\n", myPosRTK.lon, myPosRTK.lat, myPosRTK.fixed, myPosRTK.sate, myPosRTK.timeSet);
        break;
    default:
        printf("T1Act-SetPointRTK unknown error, name=%c\n", name);
        break;
    }
}

MapPoint T1Act::GetPoint(char name, int val) {
    MapPoint tmp_point;
    switch (name) {
    case 'b':
        return mapBase;
        break;
    case 'r':
        return myBase415;
        break;
    case 'a':
        return mapMax;
        break;
    case 'i':
        return mapMin;
        break;
    case '6':
        tmp_point.lon = myMower6.fx;
        tmp_point.lat = myMower6.fy;
        tmp_point.yaw = myMower6.yaw;
        return tmp_point;
        break;
    case 'M':
        tmp_point.lon = myRoboM.fx;
        tmp_point.lat = myRoboM.fy;
        tmp_point.yaw = myRoboM.yaw;
        return tmp_point;
        break;
    case 'I':
        tmp_point.lon = myRoboI.fx;
        tmp_point.lat = myRoboI.fy;
        tmp_point.yaw = myRoboI.yaw;
        return tmp_point;
        break;
    case 'V':
        tmp_point.batteryVoltage = batteryVoltage;
        //printf("T1Act-GetPoint, battery voltage: %.2fV\n", tmp_point.batteryVoltage);
        return tmp_point;
        break;
    case 'K':
        return myPosRTK;
        break;
    case 'F':
        return myPosRTKF;
        break;
    default:
        return mapBase;
        break;
    }
}

/* Return one of the 3 strings based on type
 * 2026.05.06: created by Leo
 */
void T1Act::getStrSend(char type, char* outBuffer)
{
    if (!outBuffer) return;
    memset(outBuffer, 0, LENGTHSTRSEND);
    switch (type)
    {
        case 'V': strncpy(outBuffer, meStrSendVis, LENGTHSTRSEND-1); break;
        case 'C': strncpy(outBuffer, meStrSendCmd, LENGTHSTRSEND-1); break;
        case 'S': strncpy(outBuffer, meStrSendSocket, LENGTHSTRSEND-1); break;
        default: break;
    }
}

/* clear one of the 3 strings based on type
 * 2026.05.06: created by Leo
 */
void T1Act::clearStrSend(char type)
{
    switch (type)
    {
        case 'V': memset(meStrSendVis, 0, LENGTHSTRSEND); break;
        case 'C': memset(meStrSendCmd, 0, LENGTHSTRSEND); break;
        case 'S': memset(meStrSendSocket, 0, LENGTHSTRSEND); break;
        default: break;
    }
}
/* clear one of the 3 strings based on type
 * 2026.06.15: created by Xs
 */
// log map by trail RTK's points when user remoto control
/*
void T1Act::MapByTrail() {
    char tmpstr[200];
    const char rtkfile[] = "../Map/TrailRemote.rtk";
    const char mapfile[] = "../Map/TrailBorder.map2";
    MapPoint tmp_point;
    // double lon,lat;
    static int count=0;
    // std::string str = data;
    // std::vector<std::string> list1;
    // size_t l1length=0;
    //printf("T1Act-MapByTrail_ begin: [%s]\r\n", data);

    // list1 = T1Utils::split(str,',');
    // l1length = list1.size();
    //std::cout << "T1Act-MapByTrail_ begin:"  << l1length << list1[0] << std::endl;

    //"$GNGGA,055556.00,3134.70397170,N,12020.97266526,E,5,38,0.5,26.2219,M,7.4475,M,1.0,0*6D\r"
    // tmp_point.fixed = std::stod(list1[6]);
    // tmp_point.sate = std::stoi(list1[7]);
    // tmp_point.alt = std::stoi(list1[9])*100;
    // lon = std::stod(list1[4]);
    tmp_point.fixed = myPosRTK.fixed;
    tmp_point.sate = myPosRTK.sate;
    tmp_point.alt = myPosRTK.alt;
    tmp_point.lon = myPosRTK.lon;
    tmp_point.lat = myPosRTK.lat;
    // if ( list1[5][0]=='W' ) lon*=-1;
    // lat = std::stod(list1[2]);
    // if ( list1[3][0]=='S' ) lat*=-1;
    // tmp_point.lon = T1Utils::TransLonLat(lon) * PRECISION;
    // tmp_point.lat = T1Utils::TransLonLat(lat) * PRECISION;
    
    if ( WorkStatusT == 0 ) { // get the meAngleNorth firstly
        printf("T1Act-MapByTrail step0, go forward 120cm.   myDuffMI=%d, meDiffNorth=%d, meDiffStation=%d\n", myDiffMI.yaw, meDiffNorth, meDiffStation);
        if ( meStageNorth>0 ) {
            myDiffMI.yaw = meDiffNorth;
        } else {
            myDiffMI.yaw = meDiffStation;
        }
        //WorkStatusT=1;     // this will be set in T1ActRoboAll::WorkTrail(), not here
        return;
    }
    
    if ( posFixedRP!=4 && FlagTestNight!=8 ) return;
    
    if ( WorkStatusT == 1 ) {
        if ( std::remove(rtkfile) != 0 ) {
            perror("T1Act-MapByTrail_, remove file failed.");
        } else {
            std::cout <<  "T1Act-MapByTrail_ step2a: remove file ok." << std::endl;
        }
        if ( std::remove(mapfile) != 0 ) {
            perror("T1Act-MapByTrail_, remove file failed.");
        } else {
            std::cout <<  "T1Act-MapByTrail_ step2b: remove file ok." << std::endl;
        }
        std::ofstream fileout2(mapfile, std::ios::app);
        if (!fileout2.is_open()) {
            T1Utils::LogBug("T1Act-MapByTrail_ step2c, Can not open TrailBorder.map");
            return;
        }

        pMap->myMapRemote.clear(); pMap->myMapBorder.clear(); pMap->myRoutePlan.clear();
        ((T1Map2 *)pMap)->myRoutePlanLine.clear(); ((T1Map2 *)pMap)->myRoutePlan2.clear();
        mapMax.lon = 0; mapMax.lat = 0;  mapMin.lon = 0; mapMin.lat = 0;
        mapBase.lon = tmp_point.lon;
        mapBase.lat = tmp_point.lat;
        count = 0;
        sprintf(tmpstr, "%lld,%lld,%d,%d,%d,%d",mapBase.lon,mapBase.lat,tmp_point.alt,tmp_point.fixed,tmp_point.sate,count);
        printf("T1Act-MapByTrail_ step3: %d, base(%lld,%lld), [%s]\n", count, mapBase.lon, mapBase.lat, tmpstr);
        //line = QString("T=b%1,%2\r\n").arg(mapBase.lon).arg(mapBase.lat);
        //sendDataRTKsmd(line);
        pMap->myMapRemote.push_back(tmp_point);
        fileout2 << tmpstr << std::endl;
        fileout2.close();
    }

    //QString line;
    std::ofstream fileout(rtkfile, std::ios::app);
    if (!fileout.is_open()) {
        T1Utils::LogBug("T1Act-MapByTrail_ step4a, Can not open TrailRemote.rtk");
        return;
    }
    sprintf(tmpstr, "%lld,%lld,%d,%d,%d", tmp_point.lon, tmp_point.lat, tmp_point.alt, tmp_point.fixed, tmp_point.sate);
    fileout << tmpstr << std::endl;
    fileout.close();
    
    std::ofstream fileout2(mapfile, std::ios::app);
    if (!fileout2.is_open()) {
        fileout.close();
        T1Utils::LogBug("T1Act-MapByTrail_ step4b, Can not open TrailBorder.map");
        return;
    }   
    int x,y;
    pMap->TransXY( &x, &y, tmp_point.lon, tmp_point.lat, mapBase.lon, mapBase.lat);
    printf("T1Act-MapByTrail_ step5a: xy(%d,%d), mapBase(%lld,%lld),  now(%lld,%lld)\n", x, y, mapBase.lon, mapBase.lat, tmp_point.lon, tmp_point.lat);
    float angle = myRoboM.yaw * 3.14159265 / 180;
    x -= meFact.RtkMntDiffY * sin(angle) + meFact.RtkMntDiffX * cos(angle);
    y -= meFact.RtkMntDiffY * cos(angle) - meFact.RtkMntDiffX * sin(angle);
    sprintf(tmpstr,"%d,%d,%d,%d,%d,%d,%d,%d",x,y,tmp_point.alt,tmp_point.fixed,myRoboI.yaw,myRoboI.roll,myRoboI.pitch,count);
    fileout2 << tmpstr << std::endl;
    fileout2.close();
    count ++;
    WorkStatusT++;
    printf("T1Act-MapByTrail_ step5b: xy(%d,%d), mapBase(%lld,%lld),  now(%lld,%lld), angle=%d\n", x, y, mapBase.lon, mapBase.lat, tmp_point.lon, tmp_point.lat, myRoboM.yaw);
    tmp_point.lon = x;
    tmp_point.lat = y;
    pMap->myMapRemote.push_back(tmp_point);
    sprintf(StrSocketSend,"@line,%d,%d\n",x,y);
    //printf("T1Act-MapByTrail_ step6: %d,%d fix(%d) [%s]\n", WorkStatusT, count, tmp_point.fixed, data);
    printf("T1Act-MapByTrail_ step6: %d,%d fix(%d) (%s)\n", WorkStatusT, count, tmp_point.fixed, tmpstr);

}
*/

void T1Act::MapByTrail(const char * data) {
    char tmpstr[200];
    const char rtkfile[] = "../Map/TrailRemote.rtk";
    const char mapfile[] = "../Map/TrailBorder.map2";
    MapPoint tmp_point;
    double lon,lat;
    static int count=0;
    std::string str = data;
    std::vector<std::string> list1;
    size_t l1length=0;
    //printf("T1Act-MapByTrail_ begin: [%s]\r\n", data);

    list1 = T1Utils::split(str,',');
    l1length = list1.size();
    //std::cout << "T1Act-MapByTrail_ begin:"  << l1length << list1[0] << std::endl;

    //"$GNGGA,055556.00,3134.70397170,N,12020.97266526,E,5,38,0.5,26.2219,M,7.4475,M,1.0,0*6D\r"
    tmp_point.fixed = std::stod(list1[6]);
    tmp_point.sate = std::stoi(list1[7]);
    tmp_point.alt = std::stoi(list1[9])*100;
    lon = std::stod(list1[4]);
    if ( list1[5][0]=='W' ) lon*=-1;
    lat = std::stod(list1[2]);
    if ( list1[3][0]=='S' ) lat*=-1;
    tmp_point.lon = T1Utils::TransLonLat(lon) * PRECISION;
    tmp_point.lat = T1Utils::TransLonLat(lat) * PRECISION;
    
    if ( WorkStatusT == 0 ) { // get the meAngleNorth firstly
        printf("T1Act-MapByTrail step0, go forward 120cm.   myDuffMI=%d, meDiffNorth=%d, meDiffStation=%d\n", myDiffMI.yaw, meDiffNorth, meDiffStation);
        if ( meStageNorth>0 ) {
            myDiffMI.yaw = meDiffNorth;
        } else {
            myDiffMI.yaw = meDiffStation;
        }
        //WorkStatusT=1;     // this will be set in T1ActRoboAll::WorkTrail(), not here
        return;
    }
    
    if ( posFixedRP!=4 && FlagTestNight!=8 ) return;
    
    if ( WorkStatusT == 1 ) {
        if ( std::remove(rtkfile) != 0 ) {
            perror("T1Act-MapByTrail_, remove file failed.");
        } else {
            std::cout <<  "T1Act-MapByTrail_ step2a: remove file ok." << std::endl;
        }
        if ( std::remove(mapfile) != 0 ) {
            perror("T1Act-MapByTrail_, remove file failed.");
        } else {
            std::cout <<  "T1Act-MapByTrail_ step2b: remove file ok." << std::endl;
        }
        std::ofstream fileout2(mapfile, std::ios::app);
        if (!fileout2.is_open()) {
            T1Utils::LogBug("T1Act-MapByTrail_ step2c, Can not open TrailBorder.map");
            return;
        }

        pMap->myMapRemote.clear(); pMap->myMapBorder.clear(); pMap->myRoutePlan.clear();
        ((T1Map2 *)pMap)->myRoutePlanLine.clear(); ((T1Map2 *)pMap)->myRoutePlan2.clear();
        mapMax.lon = 0; mapMax.lat = 0;  mapMin.lon = 0; mapMin.lat = 0;
        mapBase.lon = tmp_point.lon;
        mapBase.lat = tmp_point.lat;
        count = 0;
        sprintf(tmpstr, "%lld,%lld,%d,%d,%d,%d",mapBase.lon,mapBase.lat,tmp_point.alt,tmp_point.fixed,tmp_point.sate,count);
        printf("T1Act-MapByTrail_ step3: %d, base(%lld,%lld), [%s]\n", count, mapBase.lon, mapBase.lat, tmpstr);
        //line = QString("T=b%1,%2\r\n").arg(mapBase.lon).arg(mapBase.lat);
        //sendDataRTKsmd(line);
        pMap->myMapRemote.push_back(tmp_point);
        fileout2 << tmpstr << std::endl;
        fileout2.close();
    }

    //QString line;
    std::ofstream fileout(rtkfile, std::ios::app);
    if (!fileout.is_open()) {
        T1Utils::LogBug("T1Act-MapByTrail_ step4a, Can not open TrailRemote.rtk");
        return;
    }
    fileout << data << std::endl;
    fileout.close();
    
    std::ofstream fileout2(mapfile, std::ios::app);
    if (!fileout2.is_open()) {
        fileout.close();
        T1Utils::LogBug("T1Act-MapByTrail_ step4b, Can not open TrailBorder.map");
        return;
    }   
    int x,y;
    pMap->TransXY( &x, &y, tmp_point.lon, tmp_point.lat, mapBase.lon, mapBase.lat);
    printf("T1Act-MapByTrail_ step5a: xy(%d,%d), mapBase(%lld,%lld),  now(%lld,%lld)\n", x, y, mapBase.lon, mapBase.lat, tmp_point.lon, tmp_point.lat);
    float angle = myRoboM.yaw * 3.14159265 / 180;
    x -= meFact.RtkMntDiffY * sin(angle) + meFact.RtkMntDiffX * cos(angle);
    y -= meFact.RtkMntDiffY * cos(angle) - meFact.RtkMntDiffX * sin(angle);
    sprintf(tmpstr,"%d,%d,%d,%d,%d,%d,%d,%d",x,y,tmp_point.alt,tmp_point.fixed,myRoboI.yaw,myRoboI.roll,myRoboI.pitch,count);
    fileout2 << tmpstr << std::endl;
    fileout2.close();
    count ++;
    WorkStatusT++;
    printf("T1Act-MapByTrail_ step5b: xy(%d,%d), mapBase(%lld,%lld),  now(%lld,%lld), angle=%d\n", x, y, mapBase.lon, mapBase.lat, tmp_point.lon, tmp_point.lat, myRoboM.yaw);
    tmp_point.lon = x;
    tmp_point.lat = y;
    pMap->myMapRemote.push_back(tmp_point);
    sprintf(StrSocketSend,"@line,%d,%d\n",x,y);
    //printf("T1Act-MapByTrail_ step6: %d,%d fix(%d) [%s]\n", WorkStatusT, count, tmp_point.fixed, data);
    printf("T1Act-MapByTrail_ step6: %d,%d fix(%d) (%s)\n", WorkStatusT, count, tmp_point.fixed, tmpstr);

}
/* get Factory parameters setting from .UPT file
 */
void T1Act::GetFactory(const char *factoryname) {
    char filename[200];
    //char val[200]="I=1,FN,TheOne001,RA,0,0,0,0,AR,0,0,0,WD,0,0,SW,0,0,0,0,SX,0,0,SL,0,0,SC,0,0,RS,0,0,0,VA,0";
    sprintf(StrFactory,"I=1,FN,TheOne001,RA,0,0,0,0,AR,0,0,0,WD,0,0,SW,0,0,0,0,SX,0,0,SL,0,0,0,0,0,SC,0,0,RS,0,0,0,VA,0");
    //std::cout << "MyAct-GetFactory begin: " << filename << std::endl;
 
    //qDebug() << "MyAct-GetFactory begin:  " << filename;
    sprintf(filename, "./upt/%s.upt",factoryname);
    std::ifstream filein(filename);
    if (!filein.is_open() ) {
        std::cout << "MyAct-GetFactory begin: No factory file found:" << filename << std::endl;
        return;
    }
    sprintf(filename, "../Upt/Single%s.upt",factoryname);
    std::ifstream filesingle(filename);
    if (!filesingle.is_open() ) {
        std::cout << "MyAct-GetFactory open: No SingleFactory file found:" << filename << std::endl;
        return;
    }
    
    std::string line;
    std::vector<std::string> list1;
    //int tmptype = 0;

    int count = 0;
    while ( std::getline(filein,line) ) {
        count ++;
        //std::cout << "MyAct-GetFactory reading: [" << line << "]" << std::endl;
        list1 = T1Utils::split(line,' ');

        if ( list1.size() < 2 ) continue;
        if ( line.substr(0,2) == "//" )
            continue;
        if ( list1[0] == "#name" ) {
            if ( list1.size() != 2 ) continue;
            strcpy(meFact.Factory, list1[1].c_str());
            //qDebug() << "MyAct-GetFactory name: " << meFactory;
            continue;
        }
        if ( list1[0] == "#drivewheel" ) {
            if ( list1.size() != 2 ) continue;
            meFact.DriveWheel = std::stoi(list1[1]);
            //qDebug() << "MyAct-GetFactory drivewheel: " << meFactory;
            continue;
        }
        if ( list1[0] == "#HardCharging" ) {
            if ( list1.size() != 2 ) continue;
            meFact.HardCharging = std::stoi(list1[1]);
            //qDebug() << "MyAct-GetFactory HardCharging: " << meFact.HardCharging;
            continue;
        }
        if ( list1[0] == "#RoundArcle" ) {
            if ( list1.size() != 5 ) continue;
            meFact.ArcleL = std::stoi(list1[1]);
            meFact.ArcleR = std::stoi(list1[2]);
            meFact.ArcleRadius = std::stoi(list1[3]);
            meFact.ArcleDegree = std::stoi(list1[4]);
            //qDebug() << "MyAct-GetFactory Arcle: " << meFact.ArcleL << meFact.ArcleR;
            continue;
        }
        if ( list1[0] == "#AngleRot" ) {
            if ( list1.size() != 4 ) continue;
            meFact.AngleRot = std::stoi(list1[1]);
            meFact.AngleModi = std::stoi(list1[2]);
            meFact.DiffModify = std::stoi(list1[3]);
            //qDebug() << "MyAct-GetFactory NotRot: " << meFact.NotRotate << meFact.NotModify;
            continue;
        }
        if ( list1[0] == "#SpeedWalk" ) {
            if ( list1.size() != 5 ) continue;
            meFact.SpeedWalk1 = std::stoi(list1[1]);
            meFact.SpeedWalk2 = std::stoi(list1[2]);
            meFact.SpeedWalk3 = std::stoi(list1[3]);
            meFact.SpeedWalk4 = std::stoi(list1[4]);
            //printf("MyAct-GetFactory SpeedWalk: (%d,%d,%d,%d)\n", meFact.SpeedWalk1, meFact.SpeedWalk2, meFact.SpeedWalk3, meFact.SpeedWalk4);
            // << meFact.SpeedD90 << meFact.SpeedD180 << meFact.SpeedD270 << meFact.SpeedD360;
            continue;
        }
        if ( list1[0] == "#SpeedXY" ) {
            if ( list1.size() != 3 ) continue;
            meFact.SpeedXYx = std::stoi(list1[1]);
            meFact.SpeedXYy = std::stoi(list1[2]);
            //qDebug() << "MyAct-GetFactory SpeedXY: " << meFact.SpeedXYx << meFact.SpeedXYy;
            continue;
        }
        if ( list1[0] == "#StepLine" ) {
            if ( list1.size() != 6 ) continue;
            meFact.StepLinex = std::stoi(list1[1]);
            meFact.StepLiney = std::stoi(list1[2]);
            meFact.StepLinec = std::stoi(list1[3]); // the radius of the cutter, used to set the T1Map2's WidthCutter
            meFact.StepLineg = std::stoi(list1[4]); // the width of the grid, used to set the T1Map2's WidthGrid
            meFact.StepLinea = std::stoi(list1[5]); // the begin angle for RoutePlan2
            //qDebug() << "MyAct-GetFactory StepLine: " << meFact.StepLx << meFact.StepLy << meFact.StepLr;
            continue;
        }
        if ( list1[0] == "#StepClose" ) {
            if ( list1.size() != 3 ) continue;
            meFact.StepCloseS = std::stoi(list1[1]);
            meFact.StepCloseL = std::stoi(list1[2]);
            //qDebug() << "MyAct-GetFactory StepClose: " << meFact.StepCloseS << meFact.StepCloseL;
            continue;
        }
        if ( list1[0] == "#RtkSet" ) {
            if ( list1.size() != 4 ) continue;
            meFact.RtkSetFlag = std::stoi(list1[1]);
            meFact.RtkSetTime = std::stoi(list1[2]);
            meFact.RtkSetAngle = std::stoi(list1[3]);
            //qDebug() << "MyAct-GetFactory StepClose: " << meFact.RtkSetFlag << meFact.RtkSetTime << meFact.RtkSetAngle;
            continue;
        }
        if ( list1[0] == "#Vision" ) {
            if ( list1.size() != 2 ) continue;
            meFact.VisAlert = std::stoi(list1[1]);
            //qDebug() << "MyAct-GetFactory StepClose: " << meFact.VisAlert;
            continue;
        }
        if ( list1[0] == "#MagnetStop" ) {
            if ( list1.size() != 2 ) continue;
            meFact.MagnetStop = std::stoi(list1[1]);
            //printf("T1Act-GetFactory MagnetStop: %d\n", meFact.MagnetStop);
            continue;
        }
        if ( list1[0] == "#StationLength" ) { // the distance to walk when StationOut
            if ( list1.size() != 2 ) continue;
            meFact.StationLength = std::stoi(list1[1]);
            //printf("T1Act-GetFactory MagnetStop: %d\n", meFact.StationLength);
            continue;
        }
        /*if ( list1[0] == "#logtype" ) {
            if ( list1.size() !=2 ) continue;
            meLogType = std::stoi(list1[1]);
            //printf("T1Act-GetFactory LogType: %d (0=off, 1=LogLonBug, 2=LogBug, 3=both)\n", meLogType);
            continue;
        }*/
        if ( list1[0] == "#RtkMntDiff" ) {
            if ( list1.size() !=3 ) continue;
            meFact.RtkMntDiffX = std::stoi(list1[1]);
            meFact.RtkMntDiffY = std::stoi(list1[2]);
            //printf("MyAct-GetFactory RtkMntDiff: %d,%d\n", meFact.RtkMntDiffX, meFact.RtkMntDiffY);
            continue;
        }
        if ( list1[0] == "#AprilTagSize" ) {
            if ( list1.size() != 3 ) continue;
            meFact.AprilTagSize1 = std::stof(list1[1]);
            meFact.AprilTagSize2 = std::stof(list1[2]);
            //printf("MyAct-GetFactory AprilTagSize: %.3f\n", meFact.AprilTagSize);
            continue;
        }
        if ( list1[0] == "#AprilTagRange" ) {
            if ( list1.size() != 3 ) continue;
            meFact.ATagRangeA = std::stoi(list1[1]);
            meFact.ATagRangeD = std::stoi(list1[2]);
            //printf("MyAct-GetFactory AprilTagRange: %d, %d\n", meFact.ATagRangeA, meFact.ATagRangeD);
            continue;
        }
        if ( list1[0] == "#AvoidObstacle" ) {
            if ( list1.size() != 3 ) continue;
            meFact.AvoidThresh = std::stoi(list1[1]);
            meFact.AvoidFreq = std::stoi(list1[2]);
            //printf("MyAct-GetFactory AvoidObstacle: %d, %d\n", meFact.AvoidThresh, meFact.AvoidFreq);
            continue;
        }
        if ( list1[0] == "#SpeedAdjust" ) {
            if ( list1.size() != 4 ) continue;
            meFact.SpeedAdjustMax = std::stoi(list1[1]);
            meFact.SpeedAdjustMin = std::stoi(list1[2]);
            meFact.SpeedAdjustSpeed = std::stof(list1[3]);
            //printf("T1Act-GetFactory SpeedAdjust: max=%d, min=%d, speed=%f\n", meFact.SpeedAdjustMax, meFact.SpeedAdjustMin, meFact.SpeedAdjustSpeed);
            continue;
        }
        if ( list1[0] == "#TestVisionXs" ) {
            if ( list1.size() != 2 ) continue;
            meFact.TestVision = std::stoi(list1[1]);
            continue;
        }

        //qDebug() << tmptype;
    }
    filein.close();

    count = 0;
    while ( std::getline(filesingle,line) ) {
        count ++;
        //std::cout << "MyAct-GetFactory reading: [" << line << "]" << std::endl;
        list1 = T1Utils::split(line,' ');

        if ( list1.size() < 2 ) continue;
        if ( line.substr(0,2) == "//" )
            continue;
        if ( list1[0] == "#WheelDiff" ) {
            if ( list1.size() != 3 ) continue;
            meFact.WheelDiffl = std::stoi(list1[1]);
            meFact.WheelDiffr = std::stoi(list1[2]);
            //std::cout << "MyAct-GetFactorysingle WheelDiff: " << meFact.WheelDiffl << meFact.WheelDiffr << std::endl;
            continue;
        }
        if ( line.substr(0,10) == "#testnight" ) { // 1: test all the night
            SetStatus('N', std::stoi(list1[1]));
            //printf("T1Act-GetFactorysingle, testnight: %d\n", GetStatus('N'));
            continue;
        }
    }
    filesingle.close();
    sprintf(StrFactory,"I=1,FN,%s,%d,%d,RA,%d,%d,%d,%d,AR,%d,%d,%d,WD,%d,%d,SW,%d,%d,%d,%d,SX,%d,%d,SL,%d,%d,%d,%d,%d,SC,%d,%d,RS,%d,%d,%d,VA,%d,RM,%d,%d,%s\n",
            meFact.Factory, meFact.DriveWheel, meFact.MagnetStop, meFact.ArcleL, 
            meFact.ArcleR, meFact.ArcleRadius, meFact.ArcleDegree, 
            meFact.AngleRot, meFact.AngleModi, meFact.DiffModify, 
            meFact.WheelDiffl, meFact.WheelDiffr, 
            meFact.SpeedWalk1, meFact.SpeedWalk2, meFact.SpeedWalk3, meFact.SpeedWalk4,
            meFact.SpeedXYx, meFact.SpeedXYy,
            meFact.StepLinex, meFact.StepLiney, meFact.StepLinec, meFact.StepLineg, meFact.StepLinea,
            meFact.StepCloseS, meFact.StepCloseL,
            meFact.RtkSetFlag, meFact.RtkSetTime, meFact.RtkSetAngle, meFact.VisAlert,
            meFact.RtkMntDiffX, meFact.RtkMntDiffY,T1Utils::meSsid.c_str());

    printf("T1Act-GetFactory end: [%s]\n", StrFactory);
}

void T1Act::ProcessBatteryVoltage(const std::string& voltageStr) {
    try {
        const double parsedVoltage = std::stod(voltageStr);

        // 1) Raw voltage + full-charge shortcut
        batteryVoltageRaw = parsedVoltage;  // keep raw for charging detection
        if (parsedVoltage >= 21.15) {
            //printf("T1Act-$status: Battery fully charged! voltage=%.2fV >= 21.0V\n", parsedVoltage);
            FlagInCharge = 1;
        }

        // 2) Smoothed voltage for level calculation
        const double alpha = 0.6;  // Smoothing factor (0.0-1.0)
        if (batteryVoltage <= 0.0) {
            batteryVoltage = parsedVoltage;  // first sample
        } else {
            batteryVoltage = alpha * parsedVoltage + (1.0 - alpha) * batteryVoltage;
        }

        // 3) Battery level percentage (0~100)
        constexpr double fullVoltage = 21.0;   // 5S 18650 full voltage
        constexpr double emptyVoltage = 15.75;
        double ratio = (batteryVoltage - emptyVoltage) / (fullVoltage - emptyVoltage);
        int batteryLevel = static_cast<int>(ratio * 100.0);
        if (batteryLevel < 0) batteryLevel = 0;
        if (batteryLevel > 100) batteryLevel = 100;

        // 4) Real-time charging detection with dynamic thresholds
        static double lastChargeVoltage = 0.0;
        static bool hasChargeBaseline = false;
        
        // Set charging detection thresholds based on battery level
        double deltaInMin, deltaInMax;

        if (batteryLevel < 25) {
            // Battery < 25%: voltage rises 0.1~0.7V
            deltaInMin = 0.1;
            deltaInMax = 0.7;
        } else if (batteryLevel < 50) {
            // Battery < 50%: voltage rises 0.2~0.7V
            deltaInMin = 0.2;
            deltaInMax = 0.7;
        } else if (batteryLevel < 95) {
            // Battery 50%~95%: voltage rises 0.2~0.75V
            deltaInMin = 0.2;
            deltaInMax = 0.75;
        } else {
            // Battery >= 95%: voltage rises 0.1~0.4V
            deltaInMin = 0.05;
            deltaInMax = 0.4;
        }
        const double deltaOutMin = 0.1;  // Exit: Voltage drop >0.1V

        if (!hasChargeBaseline) {
            lastChargeVoltage = parsedVoltage;
            hasChargeBaseline = true;
        } else {
            double delta = parsedVoltage - lastChargeVoltage;
            if (abs(delta) > 0.05) {
                //LogLonDelta(delta);
            }
            lastChargeVoltage = parsedVoltage;

            if (FlagInCharge == 0) {
                if (delta >= deltaInMin && delta <= deltaInMax) {
                    FlagInCharge = 1;
                    IsCharing = FlagInCharge;
                    sprintf(StrCmdSend, "@CMD,g\r\n");
                    printf("T1Act-$status: IN station! battery=%d%%, delta=+%.2fV (range: %.2f~%.2fV), voltage=%.2fV\n", 
                           batteryLevel, delta, deltaInMin, deltaInMax, parsedVoltage);
                    printf("T1Act-$status: IN station! FlagInCharge=%d\n", FlagInCharge);
                }
            } else {
                if (delta < -deltaOutMin) {
                    FlagInCharge = 0;
                    hasChargeBaseline = false;
                    printf("T1Act-$status: OUT station! battery=%d%%, delta=%.2fV, voltage=%.2fV\n", 
                           batteryLevel, delta, parsedVoltage);
                }
            }
        }

        //printf("T1Act-recvLon Battery voltage: raw=%.2fV, smoothed=%.2fV, level=%d%%\n", batteryVoltageRaw, batteryVoltage, batteryLevel);
    } catch (const std::exception& e) {
        printf("\033[31m\n[3]T1Act-ProcessBatteryVoltage Battery voltage parsing error: %s, voltageStr=%s\033[0m\n", e.what(), voltageStr.c_str());
        //LogLonBug(voltageStr.c_str());
    }
}

/* reset the 415 when the IMU lost
 * 2026.04.11: creat by Leo
 * 2026.04.11: modified by Xs
 *
void T1Act::reset415() {
    printf("T1Act-reset415 begin.\n");
    //sprintf(StrCmdSend, "@CMD,R\r\n");
    //FlagReset415 = 1;
    //FlagPause = 1; // pause the work, will be reset in T1ActRoboAll::WorkReal() after the 415 is ready
    printf("T1Act-reset415 end.\n");
}*/

float T1Act::GetAprilTagSize( int type ) {
    if ( type == 1 ) {
        return meFact.AprilTagSize1;
    } else if ( type == 2 ) {
        return meFact.AprilTagSize2;
    } else {
        return 0.0;
    }
}

/* Safe-copy version string into meHardwareVersionF415
 * 2026.06.08: created by Xs
 */
void T1Act::SetHardwareVersionF415(const char *ver) {
    if (ver == nullptr) return;
    strncpy(meHardwareVersionF415, ver, sizeof(meHardwareVersionF415) - 1);
    meHardwareVersionF415[sizeof(meHardwareVersionF415) - 1] = '\0';
}

/* Get F415 hardware version string
 * 2026.06.18: created by Xs
 */
const char *T1Act::GetHardwareVersionF415() const {
    return meHardwareVersionF415;
}
/* Update T1Act state from incoming STA
 * - Store raw STAStatus
 * - Derive F415 hardware version string
 * - Process battery voltage (smoothed level for display)
 * 2026.06.02: created by Xs
 * 2026.06.05: Modified by Xs
 * 2026.06.08: Modified by Xs
 * Renamed SetSTAStatus -> UpdateFromSTA; added battery voltage processing (Xs)
 */
void T1Act::UpdateFromSTA(const STAStatus& sta) {

    mySTAStatus = sta;
    SetHardwareVersionF415(sta.version);
    
    char voltageStr[32];
    snprintf(voltageStr, sizeof(voltageStr), "%.2f", sta.batteryVoltage);
    ProcessBatteryVoltage(voltageStr);
}
/* process GNGGA data logic in T1Act
 * data already passed via SetPointRTK('R', point) to myPosRTK
 * 2026.06.15: created by Xs - separated from T1Act::recvLon()
 */
void T1Act::ProcessGNGGA( const char *data ) {
    static int count = 0;

    StataNum = myPosRTK.sate;
    rtkFixedPolaris = myPosRTK.fixed;
    posFixedRP = rtkFixedPolaris;
    //if ( FlagTestNight==1 && count%5==0 ) {
    //    sprintf(tmpstr,"T1Act-ProcessGNGGA: fixed=%d, sate=%d, lon=%lld, lat=%lld",
    //            posFixedRP, StataNum, myPosRTK.lon, myPosRTK.lat);
    //    T1Utils::LogRecord(tmpstr, "GNGGA");
    //}

    //if ( posFixedDemo != -1 ) posFixedRP = posFixedDemo;

    if ( posFixedRP == 4 ) {
        //myRTKF.lon = meScaleLon * ( myRTK.lon - mapBase.lon );
        //myRTKF.lat = meScaleLat * ( myRTK.lat - mapBase.lat );
        myPosRTKF.lon = 1 * ( myPosRTK.lon - mapBase.lon );
        myPosRTKF.lat = 1 * ( myPosRTK.lat - mapBase.lat );
        myPosRTKF.alt = myPosRTK.alt;
    }
    if ( WorkStatusT >= 0 )
        MapByTrail(data);
    
    FlagInfomation = FlagInfomation | 0x02;
    //if ( count%10 == 0 ) std::cout << "T1Act-ProcessGNGGA ok: " << posFixedRP << myPosRTK.lon << myPosRTK.lat;
    //if ( FlagTestNight==1 && count%5==0 )
        //printf("T1Act-ProcessGNGGA ok: fixed=%d,[%lld,%lld]\n", posFixedRP, myPosRTK.lon, myPosRTK.lat);

    count++;
}

