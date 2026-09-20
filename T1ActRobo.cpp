#include "T1ActRobo.hpp"
#include "T1Utils.hpp"
//#include <vector>
//#include <string>
#include <cstring>
#include <iostream>
#include <fstream>
//#include <cstdio>
#include <math.h>
#include <ctime>

T1ActRobo::T1ActRobo() {
    printf("T1ActRobo-T1ActRobo() begin......\n");
    //PolarisSet(-1); // flag the cleared sign
}

T1ActRobo::~T1ActRobo() {

}

bool T1ActRobo::IsRotationStable(float gyroZLimit, uint64_t stableTime) {
    uint64_t timenow=T1Utils::getCurrentTimeMs();
    //printf("T1ActRobo-IsRotationStable, gyroZ=%.3f\n", myRoboI.gyroZ);
    if ( myRoboI.timeIMU==0 || timenow-myRoboI.timeIMU>300 ) {
        meRotationStableStart=0;
        return false;
    }
    if ( fabs(myRoboI.gyroZ)>gyroZLimit ) {
        meRotationStableStart=0;
        return false;
    }
    if ( meRotationStableStart==0 ) {
        meRotationStableStart=timenow;
    }
    return timenow-meRotationStableStart>=stableTime;
}

/** clear the saved angle of North
 * 2026.03.15: created by Leo
 */
int T1ActRobo::AngleStationNorthClear() {
    printf("T1ActRoboAll-AngleStationNorthClear begin.\n");
    char tmpstr[200];
    sprintf(tmpstr,"../Tsk/AngleStationNorth.dat");
    std::ofstream fileout(tmpstr);
    if (!fileout.is_open()) {
        T1Utils::LogBug("T1ActRoboAll-AngleStationNorthClear, Can not open ../Tsk/AngleStationNorth.dat");
        return -1;
    }
    fileout << "meStageNorth,0,meAngleNorth,0,meDiffNorth,0,meAngleStation,0,meDiffStation,0,myRoboI,0,myMoboM,0" << std::endl;
    fileout.close();
    return 0;
}

/* North angle save
 * 2026.03.15: created by Leo
 */
int T1ActRobo::AngleStationNorthSave() {
    //printf("T1ActRoboAll-AngleStationNorthSave begin.\n");
    char tmpstr[200];
    sprintf(tmpstr,"../Tsk/AngleStationNorth.dat");
    std::ofstream fileout(tmpstr);
    if (!fileout.is_open()) {
        T1Utils::LogBug("T1ActRoboAll-AngleStationNorthSave, Can not open ../Tsk/AngleStationNorth.dat");
        return -1;
    }
    fileout << "meStageNorth," << meStageNorth << ",meAngleNorth," << meAngleNorth << ",meDiffNorth," << meDiffNorth;
    fileout << ",meAngleStation," << meAngleStation << ",meDiffStation," << meDiffStation;
    fileout << ",myRoboI," << myRoboI.yaw << ",myRobo," << myRoboM.yaw << std::endl;
    fileout.close();
    //printf("T1ActRoboAll-AngleStationNorthSave saved to %s\n", tmpstr);
    return 0;
}

/** read the saved North when power up
 * return:  0,no saved Angle North         1,get saved Anglle North
 * 2026.02.24: created by Leo
 */
int T1ActRobo::AngleStationNorthRead() {
    //printf("T1ActRobo-AngleStationNorthRead begin.\n");
    char tmpstr[200];
    sprintf(tmpstr,"../Tsk/AngleStationNorth.dat");
    std::ifstream filein(tmpstr);
    if (!filein.is_open()) {
        T1Utils::LogBug("T1ActRobo-AngleStationNorthRead step1, Can not open ../Tsk/AngleStationNorth.dat");
        return 0;
    }

    std::string line;
    std::vector<std::string> list1;
    while ( std::getline(filein,line) ) {
        list1 = T1Utils::split(line,',');
        //printf("T1ActRoboAll-AngleStationNorthRead step2, length=%ld.\n", list1.size());
        if ( list1.size() != 14 ) { // if you add some fields, notice the AngleStationNorthClear();
            printf("T1ActRobo-AngleStationNorthRead step4, saved North don't match.\n");
            break;
        }
        int stageNorth = std::stod(list1[1]);
        int angleNorth = std::stod(list1[3]);
        int diffNorth = std::stod(list1[5]);
        int angleStation = std::stod(list1[7]);
        int diffStation = std::stod(list1[9]);
        meDiffNorth = diffNorth-diffStation+meDiffStation;
        printf("T1ActRobo-AngleStationNorthRead step3, read North(%d,%d,%d) Station(%d,%d), meDiffNorth=%d\n",
            stageNorth, angleNorth, diffNorth, angleStation, diffStation, meDiffNorth);
        break; // only getline 1 line
    }
    filein.close();
    //printf("T1ActRobo-AngleStationNorthRead end. read(%s)\n", line.c_str());
    return 1;
}

/* North angle calculate
 * type = 0, clear the begin point
 * type = 1, set the begin point or 
 * 2026.03.06: reback to the old version by Leo 
 */
void T1ActRobo::NorthSet(int type) {
    static std::time_t timebegin=0, timenow=0; // timelast=0 means the first time after "the1" running
    static PointRobo pointBeginRobo;
    static MapPoint pointBeginMap;
    //printf("T1ActRobo-NorthSet begin, type=%d, rtkFixed=%d\n", type, rtkFixedPolaris);

    if ( type == 0 ) {
        //printf("T1ActRobo-NorthSet step1, type=%d, pointBeginRobo(%.1f,%.1f,%d)\n", type, pointBeginRobo.fx, pointBeginRobo.fy, pointBeginRobo.yaw );
        pointBeginRobo.yaw = -1000; // -1000 means cleared sign
        return;
    }
    if ( rtkFixedPolaris!=4 && FlagTestNight!=8 ) return;
    //if ( rtkFixedPolaris < 4 ) return;

    timenow = time(NULL); 
    //printf("T1ActRobo-NorthSet step2, type=%d, myRoboM(%.1f,%.1f,%d), time(%lld,%lld)\n", type, myRoboM.fx, myRoboM.fy, myRoboM.yaw, (long long)timenow, (long long)timebegin);

    if ( pointBeginRobo.yaw == -1000 ) {
        pointBeginRobo.fx = myRoboM.fx;
        pointBeginRobo.fy = myRoboM.fy;
        pointBeginRobo.yaw = myRoboM.yaw;
        pointBeginMap.lon = myPosRTK.lon;
        pointBeginMap.lat = myPosRTK.lat;
        timebegin = timenow;
        //printf("T1ActRobo-NorthSet step3, save the starting point: RTK(lon=%lld, lat=%lld), RoboYaw=%d, IMU=%d\n", myPosRTK.lon, myPosRTK.lat, myRoboM.yaw, myRoboI.yaw);
    } else {
        //Check One:Whether the heading angle is remains straight (change less than 2 degrees)
        if ( abs(myRoboM.yaw - pointBeginRobo.yaw) > 2 ) {
            //printf("T1ActRobo-NorthSet step4 The yaw change too much, reset.\n");
            pointBeginRobo.yaw = -1000; // reset the flag
            return;
        }
        //Check Two:Whether the distance is enough (>50 units, about 0.5 meters)
        int x,y;
        //printf("\033[34mTransXY input: myPosRTK=(%lld,%lld), pointBeginMap=(%lld,%lld)\033[0m\n",myPosRTK.lon, myPosRTK.lat, pointBeginMap.lon, pointBeginMap.lat);
        pMap->TransXY(&x, &y, myPosRTK.lon, myPosRTK.lat, pointBeginMap.lon, pointBeginMap.lat);
        //printf("\033[34mTransXY output: x=%d, y=%d\033[0m\n", x, y);
        double distance = sqrt( (double)(x*x + y*y) );
        //printf("T1ActRobo-NorthSet step5, check distance: x=%d, y=%d, distance=%.1f, time difference=%lld seconds\n", x, y, distance, (long long)(timenow - timebegin));    
        if ( distance > 10*(meStageNorth+1) ) {
            //double angleGPS_rad = -1 * atan2((double)x, (double)y);
            double angleGPS_rad = atan2((double)x, (double)y);
            meAngleNorth = 180*(angleGPS_rad/3.14159265);//Calculate the true north Angle
            if (meAngleNorth < 0) meAngleNorth += 360.0;
            // The difference between the real northbound and imu
            meDiffNorth = meAngleNorth - myRoboI.yaw; // ??? maybe need set myDiffMI.yaw here
            if ( meStageNorth<9 ) meStageNorth++;
            char tmpstr[200];
            sprintf(tmpstr,"T1ActRobo-NorthSet, AngleNorth:(%d,%d,%d), AngleStation:(%d,%d), begin:(%.1f,%.1f,%d), now:(%.1f,%.1f,%d)\n",
                meStageNorth, meAngleNorth, meDiffNorth, meAngleStation, meDiffStation, pointBeginRobo.fx, pointBeginRobo.fy, pointBeginRobo.yaw, myRoboM.fx, myRoboM.fy, myRoboM.yaw);
            LogNorth(tmpstr);
            AngleStationNorthSave();
            //printf("T1ActRobo-NorthSet step6, north set to:(%d,%d,%d), dist:%.1f, Begin(%lld,%lld,%d), now(%lld,%lld), myRoboM(%d,%d,%d)\n", meStageNorth, meAngleNorth, meDiffNorth,
            //    distance, pointBeginMap.lon, pointBeginMap.lat, pointBeginRobo.yaw, myPosRTK.lon, myPosRTK.lat, x, y, myRoboM.yaw);
            printf("T1ActRobo-NorthSet step6, north set || myRoboI(%d) AgN-%d:%d,%d, dist:%.1f, Begin(%d), myRoboM(%d,%d,%d)\n", myRoboI.yaw, meStageNorth, meAngleNorth, meDiffNorth,
                distance, pointBeginRobo.yaw, x, y, myRoboM.yaw);
        }
    }

    //printf("T1ActRobo-NorthSet end, north set to: Fixed=%d, timebegin=%ld\n", rtkFixedPolaris, timebegin);
    rtkFixedPolaris = 0;
    timebegin = timenow;
}

/* 2026.05.16: moved from T1Act.cpp by Leo
 */
void T1ActRobo::LogNorth(const char * data) {
    // Use unified T1Utils time functions
    std::string mydate = T1Utils::getCurrentYYYYMMDD();
    std::string hhmmss = T1Utils::getCurrentHHMMSS();
    uint64_t timeUs = T1Utils::getCurrentTimeUs();

    char mytime[30];
    snprintf(mytime, sizeof(mytime), "[%s.%06ld]", hhmmss.c_str(), timeUs % 1000000);
    std::string ssid = T1Utils::meSsid;
    char filename[80];
    int ret = snprintf(filename, sizeof(filename), "../Log/%sNorth%s.log",
                       ssid.c_str(), mydate.c_str());
    if (ret < 0 || ret >= (int)sizeof(filename)) {
        std::cerr << "T1ActRobo-LogNorth, filename too long" << std::endl;
        return;
    }

    std::ofstream fileout(filename, std::ios::app);
    if (!fileout.is_open()) {
        std::cerr << "T1Act-LogNorth, Can not open log file:" << filename << std::endl;
        return;
    }
    fileout << "T1ActRobo:" << mytime << "[" << data << "]" << std::endl;
    fileout.close();
}

/* Change the Robot's position after one step act by wheel odometry
 * 2023.05.01: created by Leo
 * 2026.05.05: renamed by Leo
 */
void T1ActRobo::PosiSetByWheelO(char act, int val, int ret) {
    uint64_t timenow,timediff;
    timenow = T1Utils::getCurrentTimeMs();
    int debug=0;
    int angle=0, radius=0;
    //if (debug) printf("T1ActRobo-PosiSetByWheelO begin: (%d,%d), cmd=[%c,%d,%d], myRoboM(%lld,%lld,%d), IMU:%d\n", meWorkline, meWorkstep, act, val, ret, myRoboM.fx, myRoboM.fy, myRoboM.yaw, myRoboI.yaw);

    switch (act) {
    case 'w':
    case 'W':
        myRoboM.fx += (ret*meFact.SpeedWalk1/1000) * sin( myRoboM.yaw * 3.14159265 / 180 );
        myRoboM.fy += (ret*meFact.SpeedWalk1/1000) * cos( myRoboM.yaw * 3.14159265 / 180 );
        //myRoboM.fx += val * sin( myRoboM.yaw * 3.14159265 / 180 );
        //myRoboM.fy += val * cos( myRoboM.yaw * 3.14159265 / 180 );
        if (debug) printf("T1ActRobo-PosiSetByWheelO W/w, ret=%d, range=%d, yaw=%d\n", ret, (ret*meFact.SpeedWalk1/1000), myRoboM.yaw);
        break;
    case 'x':
    case 'X':
        myRoboM.fx += (ret*meFact.SpeedWalk2/1000) * sin( myRoboM.yaw * 3.14159265 / 180 );
        myRoboM.fy += (ret*meFact.SpeedWalk2/1000) * cos( myRoboM.yaw * 3.14159265 / 180 );
        if (debug) printf("T1ActRobo-PosiSetByWheelO X/x, ret=%d, range=%d, yaw=%d\n", ret, (ret*meFact.SpeedWalk1/1000), myRoboM.yaw);
        //myRoboM.yaw = myRoboI.yaw;
        break;
    case 'S':
    case 's':
        myRoboM.fx -= (ret*meFact.SpeedWalk4/1000) * sin( myRoboM.yaw * 3.14159265 / 180 );
        myRoboM.fy -= (ret*meFact.SpeedWalk4/1000) * cos( myRoboM.yaw * 3.14159265 / 180 );
        break;
    case 'Q':
        angle = ret * 180.0 / TIMEARCLE ;
        radius = meFact.ArcleRadius;
        // calculate the center point of the circle
        //myRoboM.fx -= radius * cos( myRoboM.yaw * 3.14159265 / 180 );
        //myRoboM.fy += radius * sin( myRoboM.yaw * 3.14159265 / 180 );
        //myRoboM.fx -= radius * cos( yawbegin * 3.14159265 / 180 );
        //myRoboM.fy += radius * sin( yawbegin * 3.14159265 / 180 );
        // calculate the stop point in the circle
        myRoboM.fx = myRoboC.fx + meFact.ArcleRadius * cos( myRoboM.yaw * 3.14159265 / 180 );
        myRoboM.fy = myRoboC.fy - meFact.ArcleRadius * sin( myRoboM.yaw * 3.14159265 / 180 );
        if (debug) printf("T1ActRobo-PosiSetByWheelO 'Q': radius=%d, act=[%c,%d], RoboM(%.1f,%.1f,%d)\n", meFact.ArcleRadius, act,val, myRoboM.fx, myRoboM.fy, myRoboM.yaw);
        break;
    case 'E':
        angle = ret * 180.0 / TIMEARCLE ;
        radius = meFact.ArcleRadius;
        // calculate the center point of the circle
        //myRoboM.fx -= radius * cos( myRoboM.yaw * 3.14159265 / 180 );
        //myRoboM.fy += radius * sin( myRoboM.yaw * 3.14159265 / 180 );
        //myRoboM.fx -= radius * cos( yawbegin * 3.14159265 / 180 );
        //myRoboM.fy += radius * sin( yawbegin * 3.14159265 / 180 );
        // calculate the stop point in the circle
        myRoboM.fx = myRoboC.fx - meFact.ArcleRadius * cos( myRoboM.yaw * 3.14159265 / 180 );
        myRoboM.fy = myRoboC.fy + meFact.ArcleRadius * sin( myRoboM.yaw * 3.14159265 / 180 );
        if (debug) printf("T1ActRobo-PosiSetByWheelO 'E': radius=%d, act=[%c,%d], RoboM(%.1f,%.1f,%d)\n", meFact.ArcleRadius, act,val, myRoboM.fx, myRoboM.fy, myRoboM.yaw);
        break;
    case 'c':
    case 'C':
        //myRoboM.yaw += val;
        //myRoboM.yaw = myRoboI.yaw;
        break;
    case 'z':
    case 'Z':
        //myRoboM.yaw -= val;
        //myRoboM.yaw = myRoboI.yaw;
        break;
    default:
        printf("PosiSetByWheelO unknown act: %d, %d, Act(%c)\n", meWorkline, meWorkstep, act);
        break;
    }
    myRoboM.flagPosSetBy = 'W';
    myRoboM.timeSet=timenow;
    //if (debug) printf("T1ActRobo-PosiSetByWheelO end. myRoboM:(%lld,%lld,%d), IMU:(%d), DiffMI:(%d)\n", myRoboM.fx, myRoboM.fy, myRoboM.yaw, myRoboI.yaw, my-DiffMI.yaw);
    //if (debug) printf("T1ActRobo-PosiSetByWheelO end."
}
/* Judge if a bang is in front
 * return:  0:safty,   1:danger
 * return:  >1: how to rotate to left the obstacle,  not used now
 */
/*int MyMap::RoboBangJudge() {
    //printf("T1ActRoboRobotBangJudge begin: " << meWorkline << meWorkstep << cmd << val << countmax << count;
    //printf("T1ActRoboRobotBangJudge end."
    if ( FlagBangRotating == 1 )
        return 0;
    return 0;
}

/* The New Version for Robot Act in each time split, only get system time, no count split
 * return:  -1:not work,   0:working,    >0:time of working (ms, impossible large than 30 seconds)
 */
int T1ActRobo::RoboAct(char act, int val) {
    int debug=0;
    static char cmd[100]="--";
    static uint64_t timenow=0, timemax=0, timewalk=0, timeold=0;
    static int FlagSend=0;
    timenow = T1Utils::getCurrentTimeMs();
    //if (debug) printf("T1ActRobo-RobotAct begin: diff=%ld, timeold=%ld, timenow=%ld\n", timenow-timeold, timeold, timenow);
    timeold = timenow;
    //long microseconds = ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
    //if (debug) printf("T1ActRobo-RobotAct begin: %d,%d, Act('%c',%d), cmd=[%s], time:(%ld,%ld,%ld)\n", meWorkline, meWorkstep, act, val, cmd, timewalk, timenow, timemax);

    if ( act != '=' ) { // first enter to analysis command action
        switch (act) {
        case '0':
            timemax=0; timewalk=0;
            break;
        case 'I': // information sending setting
            sprintf(cmd,"%c=%d", act, val);
            timemax=0; timewalk=0;
            sprintf(StrCmdSend,"@CMD,%s\r\n", cmd);
            printf("T1ActRobo-RoboAct(%c,%d), cmd=[%s]\n", act, val, cmd);
            break;
        case 'g': // stop
            sprintf(cmd,"%c", act);
            timemax=0; timewalk=0;
            sprintf(StrCmdSend,"@CMD,%s\r\n", cmd);
            break;
        case 'w':
        case 'W': // walk forward
            sprintf(cmd,"%c=%d", act, WALKPWM);
            timewalk = 1000.0 * val / meFact.SpeedWalk1 + 50; // the 50 is for too short distance
            timemax = timenow + timewalk;
            sprintf(StrCmdSend,"@CMD,%s\r\n",cmd);
            break;
        case 'v':
        case 'V': // walk forward
            sprintf(cmd,"w=%d", 40);
            timewalk = 1000.0 * val / 23.0;
            timemax = timenow + timewalk;
            sprintf(StrCmdSend,"@CMD,%s\r\n",cmd);
            break;
        case 'x':
        case 'X':
            if ( val > NOTMODIFY ) {
                val = val+1-NOTMODIFY;
            } else if ( val < (-1)*NOTMODIFY ) {
                val = val-1+NOTMODIFY;
            } else {
                val = 0;
            }
            sprintf(cmd,"%c=%d", act, val);
            timewalk = 400.0 * abs(val);
            timemax = timenow + timewalk;
            sprintf(StrCmdSend,"@CMD,%s\r\n",cmd);
            break;
        case 'S': // walk backward
            sprintf(cmd,"%c", act);
            timewalk = 1000.0 * val / meFact.SpeedWalk4;
            timemax = timenow + timewalk;
            sprintf(StrCmdSend,"@CMD,%s\r\n",cmd);
            break;
        case 's': // walk backward (slow speed, 50% of normal)
            sprintf(cmd,"%c=%d", act, WALKBACK);
            timewalk = 1000.0 * val / meFact.SpeedWalk4 + 50;  // the 50 is for too short distance
            //timewalk = 1000.0 * val / meFact.SpeedWalk4;
            //printf("T1ActRobo-RoboAct step1 s: val=%d, timewalk=%ld,%d\n", val, timewalk,meFact.SpeedWalk4);
            //printf("T1ActRobo-RoboAct step1 s: val=%d, timewalk=%ld\n", val, timewalk);
            timemax = timenow + timewalk;
            sprintf(StrCmdSend,"@CMD,%s\r\n",cmd);
            break;
        case 'r':// walk forward
            sprintf(cmd,"w=%d", WALKSLOW);
            timewalk = 1000.0 * val / (meFact.SpeedWalk4 * 20.0 / WALKSLOW); 
            printf("T1ActRobo-RoboAct step1 r: val=%d, timewalk=%ld\n", val, timewalk);
            timemax = timenow + timewalk;
            //sprintf(StrCmdSend,"@CMD,w\r\n");
            sprintf(StrCmdSend,"@CMD,%s\r\n",cmd);
            break;
        case 'z': // turn left 10 degree, or the val degree
            //val= val+1-NOTROT;
            sprintf(cmd,"%c=%d", act, val);
            timewalk = 1000 + 800.0 * val /26;
            timemax = timenow + timewalk;
            sprintf(StrCmdSend,"@CMD,g\r\n");
            FlagSend = 1;
            //sprintf(StrCmdSend,"@CMD,%s\r\n",cmd);
            break;
        case 'Z': // turn left 10 degree, or the val degree
            //val= val+1-NOTROT;
            sprintf(cmd,"%c=%d", act, val);
            timewalk = 3000 + 1000.0 * val /SPEEDROT;
            timemax = timenow + timewalk;
            sprintf(StrCmdSend,"@CMD,g\r\n");
            FlagSend = 1;
            //sprintf(StrCmdSend,"@CMD,%s\r\n",cmd);
            break;
        case 'c': // turn right 10 degree, or the val degree
            //val= val+1-NOTROT;
            sprintf(cmd,"%c=%d", act, val);
            timewalk = 1000 + 800.0 * val /26;
            timemax = timenow + timewalk;
            sprintf(StrCmdSend,"@CMD,g\r\n");
            FlagSend = 1;
            //sprintf(StrCmdSend,"@CMD,%s\r\n",cmd);
            break;
        case 'C': // turn right 10 degree, or the val degree
            //val= val+1-NOTROT;
            sprintf(cmd,"%c=%d", act, val);
            timewalk = 3000 + 1000.0 * val /SPEEDROT;
            timemax = timenow + timewalk;
            sprintf(StrCmdSend,"@CMD,g\r\n");
            FlagSend = 1;
            //sprintf(StrCmdSend,"@CMD,%s\r\n",cmd);
            break;
        case 'Q':
            sprintf(cmd,"%c=%d", act, val);
            sprintf(StrCmdSend,"@CMD,%s\r\n",cmd);
            timewalk = 3000 + 1000.0 * val / 10;
            timemax = timenow + timewalk;
            printf("T1ActRobo-RoboAct step1 Q %c=%d\n",act,val);
            break;
        case 'E':
            sprintf(cmd,"%c=%d", act, val);
            sprintf(StrCmdSend,"@CMD,%s\r\n",cmd);
            timewalk = 3000 + 1000.0 * val / 10;
            timemax = timenow + timewalk;
            printf("T1ActRobo-RoboAct step1 E %c=%d\n",act,val);
            break;
        default:
            timemax=0; timewalk=0;
            printf("T1ActRobo-RobotAct step1 error, unknown Act([%c],%d)\n", act, val);
            return -1;
            break;
        }
        //printf("T1ActRobo-RobotAct step1: Act([%c],%d), cmd:[%s], time:(%ld,%ld,%ld)\n", act, val, cmd, timewalk, timenow, timemax);
        return 0; // ret=0;
    } else { // enter when act is '=', loop working
        int type=0;
        if ( cmd[0]=='W' || cmd[0]=='w' || cmd[0]=='S' ) // calculat the Polaris from RTK
            type=1;
        NorthSet(type);
        if ( timewalk == 0 ) {
            //printf("T1ActRobo-RobotAct Action Done, cmd=[%s], timewalk=%ld\n", cmd, timewalk);
            return -1;
        }
        if ( timenow > timemax ) {
            if (debug>1) printf("T1ActRobo-RobotAct time2max: cmd[%c],(%ld,%ld,%ld)\n", cmd[0], timewalk, timenow, timemax);
            if ( cmd[0]=='w'|| cmd[0]=='v' ) {
                sprintf(StrCmdSend,"@CMD,g\r\n");
                int ret = timewalk;
                timemax=0; timewalk=0;
                return ret;
            } else if (cmd[0]=='W' || cmd[0]=='v'  ) {//|| cmd[0]=='s' || cmd[0]=='X' || cmd[0]=='x' || cmd[0]=='q')
                int ret =  timewalk;
                timemax=0; timewalk=0;
                return ret;
            } else if (cmd[0]=='S') {//|| cmd[0]=='S')
                sprintf(StrCmdSend,"@CMD,g\r\n");
                int ret = timewalk;
                timemax=0; timewalk=0;
                return ret;
            } else if (cmd[0]=='s') {//|| cmd[0]=='s')
                //sprintf(StrCmdSend,"@CMD,g\r\n");
                int ret = timewalk;
                timemax=0; timewalk=0;
                return ret;
            } else if ( cmd[0]=='X' ) {//|| cmd[0]=='x' ) {
                int ret = timewalk;
                printf("T1ActRobo-RobotAct X time2max: (%ld,%ld,%ld)\n", timewalk, timenow, timemax);
                timemax=0; timewalk=0;
                return ret;
            } else if ( cmd[0]=='Z' || cmd[0]=='C' ) {
                sprintf(StrCmdSend,"@CMD,g\r\n");
                int ret = timewalk;
                printf("T1ActRobo-RobotAct Z/C time2max: (%ld,%ld,%ld)\n", timewalk, timenow, timemax);
                timemax=0; timewalk=0;
                return ret;
            } else if ( cmd[0]=='z' || cmd[0]=='c' ) {
                sprintf(StrCmdSend,"@CMD,g\r\n");
                int ret = timewalk;
                printf("T1ActRobo-RobotAct z/c time2max: (%ld,%ld,%ld)\n", timewalk, timenow, timemax);
                timemax=0; timewalk=0;
                return ret;
            } else if ( cmd[0]=='Q' || cmd[0]=='E' ) {
                sprintf(StrCmdSend,"@CMD,g\r\n");
                int ret = timewalk;
                printf("T1ActRobo-RobotAct %c time2max: (%ld,%ld,%ld)\n", cmd[0], timewalk, timenow, timemax);
                timemax=0; timewalk=0;
                return ret;
            }
            //printf("T1ActRobo-RobotAct time2max error: cmd:[%s}+], time:(%ld,%ld,%ld)\n", cmd, timewalk, timenow, timemax);            
            return -1;
        }
        //printf("T1ActRobo-RobotAct step2, cmd=[%s], FlagSend=%d\n", cmd, FlagSend);
        if ( FlagSend > 0 ) {
            if ( cmd[0]=='z' || cmd[0]=='c' ) {
                sprintf(StrCmdSend,"@CMD,%s\r\n",cmd);
                FlagSend = 0;
            } else if ( cmd[0]=='Z' || cmd[0]=='C' ) {
                sprintf(StrCmdSend,"@CMD,%s\r\n",cmd);
                FlagSend = 0;
            } else if ( cmd[0]=='q' || cmd[0]=='e' ) { // 'q','e' may be 1 wheel turn if val>5
                FlagSend++;
                if ( FlagSend == 2 ) {
                    sprintf(StrCmdSend,"@CMD,%c=1\r\n", cmd[0]);
                } else if ( FlagSend == DELAYPRE ) {
                    sprintf(StrCmdSend,"@CMD,%s\r\n",cmd);
                    FlagSend = 0;
                }
            } else { // never come here
                printf("T1ActRobo-RobotAct step2 error, should never come here, cmd=[%s], FlagSend=%d\n", cmd, FlagSend);
                FlagSend = 0;
            }
            //printf("T1ActRobo-RobotAct step2, cmd=[%s], FlagSend=%d\n", cmd, FlagSend);
        }
        //printf("T1ActRobo-RobotAct step3, cmd=[%s], FlagStop=%d\n", cmd, FlagStop);
        switch (FlagStop) {
        case 0:
            break;
        case 1:
            if ( cmd[0]!='z' && cmd[0]!='Z' ) {
                if (debug>0) printf("T1ActRobo-RobotAct step3 error, cmd=[%s], FlagStop=%d\n", cmd, FlagStop);
                //FlagStop = 0;
            } else {
                int ret = (timenow+timewalk-timemax);
                if (debug>1) printf("T1ActRobo-RobotAct step3-z: FlagStop=%d, ret=%d, timewalk=%ld, cmd=[%s]\n", FlagStop, ret, timewalk, cmd);
                timemax=0; timewalk=0;
                FlagStop = 0;
                return ret;
            }
            break;
        case 2:
            if ( cmd[0]!='c' && cmd[0]!='C' ) {
                if (debug>0) printf("T1ActRobo-RobotAct step3 error, cmd=[%s], FlagStop=%d\n", cmd, FlagStop);
                //FlagStop = 0;
            } else {
                int ret = (timenow+timewalk-timemax);
                if (debug>1) printf("T1ActRobo-RobotAct step3-c: FlagStop=%d, ret=%d, timewalk=%ld, cmd=[%s]\n", FlagStop, ret, timewalk, cmd);
                timemax=0; timewalk=0;
                FlagStop = 0;
                return ret;
            }
            break;
        case 3:
            if ( cmd[0]!='X' ) {
                printf("T1ActRobo-RobotAct step3 error, cmd=[%s], FlagStop=%d\n", cmd, FlagStop);
                //FlagStop = 0;
            } else {
                int ret = (timenow+timewalk-timemax);
                if (debug>1) printf("T1ActRobo-RobotAct step3-X: FlagStop=%d, ret=%d, timewalk=%ld, cmd=[%s]\n", FlagStop, ret, timewalk, cmd);
                timemax=0; timewalk=0;
                FlagStop = 0;
                return ret;
            }
            break;
        case 4:
            if ( cmd[0]!='X' ) {
                printf("T1ActRobo-RobotAct step3 error, cmd=[%s], FlagStop=%d\n", cmd, FlagStop);
                //FlagStop = 0;
            } else {
                int ret = (timenow+timewalk-timemax);
                if (debug>1) printf("T1ActRobo-RobotAct step3-X: FlagStop=%d, ret=%d, timewalk=%ld, cmd=[%s]\n", FlagStop, ret, timewalk, cmd);
                timemax=0; timewalk=0;
                FlagStop = 0;
                return ret;
            }
            break;
        case 5:
        case 8:
        case 9:
            if ( cmd[0]=='W' || cmd[0]=='w' || cmd[0]=='X' || cmd[0]=='x' || cmd[0]=='Q' || cmd[0]=='E' ) {
                int ret = (timenow+timewalk-timemax);
                if (debug>1) printf("T1ActRobo-RobotAct step3-WxXxQE: FlagStop=%d, ret=%d, timewalk=%ld, cmd=[%s]\n",
                    FlagStop, ret, timewalk, cmd);
                timemax=0; timewalk=0;
                return ret;
            } else if ( cmd[0]=='c' || cmd[0]=='z' || cmd[0]=='C' || cmd[0]=='Z' ) {
                ;
            } else {
                printf("T1ActRobo-RobotAct step3 error, cmd=[%s], FlagStop=%d\n", cmd, FlagStop);
                //FlagStop = 0;
            }
            break;
        case 6:
            if ( cmd[0]!='Q' ) {
                printf("T1ActRobo-RobotAct step3 error, cmd=[%s], FlagStop=%d\n", cmd, FlagStop);
            } else {
                int ret = (timenow+timewalk-timemax);
                if (debug>1) printf("T1ActRobo-RobotAct step3-%c: FlagStop=%d, ret=%d, timewalk=%ld, cmd=[%s]\n", cmd[0], FlagStop, ret, timewalk, cmd);
                timemax=0; timewalk=0;
                FlagStop = 0;
                FlagBangRotating = 0;
                return ret;
            }
            break;
        case 7:
            if ( cmd[0]!='E' ) {
                printf("\033[31mT1ActRobo-RobotAct step3 error, cmd=[%s], FlagStop=%d\033[0m\n", cmd, FlagStop);
            } else {
                int ret = (timenow+timewalk-timemax);
                if (debug>1) printf("T1ActRobo-RobotAct step3-%c: FlagStop=%d, ret=%d, timewalk=%ld, cmd=[%s]\n", cmd[0], FlagStop, ret, timewalk, cmd);
                timemax=0; timewalk=0;
                FlagStop = 0;
                FlagBangRotating = 0;
                return ret;
            }
            break;
        default:
            if (debug>0) printf("T1ActRobo-RobotAct step3 error, cmd=[%s], FlagStop=%d\n", cmd, FlagStop);
            break;
        }
        //printf("T1ActRobo-RobotAct step4: cmd:[%s], Act('%c'), time=(%ld,%ld,%ld)\n", cmd, act, timewalk, timenow, timemax );
        if ( FlagBangRotating == 2 ) { // || FlagBangRotating == 4 ) {
            if ( cmd[0] == 'Q' ) {
                // calculate the current point in the circle
                myRoboM.fx = myRoboC.fx + meFact.ArcleRadius * cos( (myRoboM.yaw) * 3.14159265 / 180 );
                myRoboM.fy = myRoboC.fy - meFact.ArcleRadius * sin( (myRoboM.yaw) * 3.14159265 / 180 );
            }else if ( cmd[0] == 'E' ) {
                // calculate the current point in the circle
                myRoboM.fx = myRoboC.fx - meFact.ArcleRadius * cos( (myRoboM.yaw) * 3.14159265 / 180 );
                myRoboM.fy = myRoboC.fy + meFact.ArcleRadius * sin( (myRoboM.yaw) * 3.14159265 / 180 );
            }
            if (debug>1) printf("T1ActRobo-RobotAct step5: RoboM(%.1f,%.1f,%d), RoboC(%.1f,%.1f,%d)\n", myRoboM.fx, myRoboM.fy, myRoboM.yaw, myRoboC.fx, myRoboC.fy, myRoboC.yaw);
            return 0;
        }
        //printf("T1ActRobo-RobotAct step6: cmd:[%s], Act('%c'), time=(%ld,%ld,%ld)\n", cmd, act, timewalk, timenow, timemax );
        return 0;
    }
    if (debug>0) printf("T1ActRoboRobotAct end error.");
    return -1;
}

/* Set the position of mower by Visual fixed date
 * 2026.04.15: created by Leo
 * 2026.04.16: modified by BY
 * 2026.05.05: renamed by Xs
 */

void T1ActRobo::PosiSetByVisA() {
     int debug=0;
     char tmpstr[200];
     if (debug>1) printf("T1ActRobo-PosiSetByVisA begin, myRoboM=(%.1f,%.1f,%d), myRoboVA=(%.1f)\n", myRoboM.fx, myRoboM.fy, myRoboM.yaw,myRoboVA.fy);
     static uint64_t timelast = 0;
     uint64_t timenow, timediff;
     timenow = T1Utils::getCurrentTimeMs();
     timediff = timenow - timelast;
     int gainKarlman = 10;
     if (timediff < meKarlmanA) // Default meKarlmanA value
         gainKarlman = timediff * 10 / meKarlmanA + 1;
     // Get coordinates from myPosVisA (already in map coordinates)
     sprintf(tmpstr,"T1ActRobo-PosiSetByVisA step1, myRoboM=(%.1f,%.1f,%d)", myRoboM.fx, myRoboM.fy, myRoboM.yaw);
     if (debug>1) T1Utils::LogRecord(tmpstr, "PosiSetByVisA", 1);
     float x = myPosVisA.fx;
     float y = myPosVisA.fy;
     sprintf(tmpstr,"T1ActRobo-PosiSetByVisA step2, myPosVisA=(%.1f,%.1f,%d)", x, y, myRoboM.yaw);
     if (debug>1) T1Utils::LogRecord(tmpstr, "PosiSetByVisA", 1);
     // Calculate the mount diff of the camera
     float angle = myRoboM.yaw * 3.14159265 / 180;
     x -= meFact.RtkMntDiffY * sin(angle) + meFact.RtkMntDiffX * cos(angle);
     y -= meFact.RtkMntDiffY * cos(angle) - meFact.RtkMntDiffX * sin(angle);

    sprintf(tmpstr,"T1ActRobo-PosiSetByVisA step3, myRoboM(%.1f,%.1f,%d)->myPosVisA(%.1f,%.1f) MntDiffXY=(%d,%d)",
        myRoboM.fx,myRoboM.fy,myRoboM.yaw, x,y, meFact.RtkMntDiffX,meFact.RtkMntDiffY);
    if (debug>1) T1Utils::LogRecord(tmpstr, "PosiSetByVisA",1);
    //T1Utils::LogBug(tmpstr);

     // Apply Kalman filter
     //myRoboM.fx += gainKarlman / 10.0 * (x - myRoboM.fx);
     //myRoboM.fy += gainKarlman / 10.0 * (y - myRoboM.fy);
     myRoboM.fx = x; myRoboM.fy = y;
     // Attention!!! should think about the robot is walking after the visual position is set
     timelast = timenow;
     myRoboM.flagPosSetBy = 'A';
     myRoboM.timeSet = myRoboVA.timeSet;
     //myRoboM.timeSet = timenow;
     //printf("\033[32mT1ActRobo-PosiSetByVisA step4, gainKarlman=%d, timediff=%ld, timelast=%ld, timenow=%ld\033[0m\n", gainKarlman, timediff, timelast, timenow);
     if (debug>1) printf("T1ActRobo-PosiSetByVisA end, myRoboM=(%.1f,%.1f,%d)\n", myRoboM.fx, myRoboM.fy, myRoboM.yaw);
}

/* Set the position of mower by RTK's fixed date
 * 2026.04.15: created by Leo
 */
void T1ActRobo::PosiSetByRTK() {
    static uint64_t timenow=0, timelast=0, timediff=0;
    timenow = T1Utils::getCurrentTimeMs();
    timediff = timenow - timelast;
    int gainKarlman = 10;
    if ( timediff<meKarlmanR )
        gainKarlman = timediff*10/meKarlmanR + 1;
    printf("T1ActRobo-PosiSetByRTK begin, gainKarlman=%d diff=%ld, timelast=%ld, timenow=%ld\n", gainKarlman, timediff, timelast, timenow);

    int x,y;
    pMap->TransXY( &x, &y, myPosRTK.lon, myPosRTK.lat, mapBase.lon, mapBase.lat );
    //printf("T1ActRobo-PosiSetByRTK step2, x=%d,y=%d,  myPosRTK.lon=%lld,myRoboM.lat=%lld,  ",x,y,myPosRTK.lon,myPosRTK.lat);
    //printf("mapBase.lon=%lld,mapBase.lat=%lld\n",mapBase.lon,mapBase.lat);
    // calculate the monut diff of the RTK antenna
    float angle = myRoboM.yaw * 3.14159265 / 180;
    x -= meFact.RtkMntDiffY * sin(angle) + meFact.RtkMntDiffX * cos(angle);
    y -= meFact.RtkMntDiffY * cos(angle) - meFact.RtkMntDiffX * sin(angle);
    //myRoboM.fx = x;
    //myRoboM.fy = y;
    myRoboM.fx += gainKarlman/10.0 * ( x - myRoboM.fx );
    myRoboM.fy += gainKarlman/10.0 * ( y - myRoboM.fy );
    timelast = timenow;
    myRoboM.flagPosSetBy = 'R';
    myRoboM.timeSet=timenow;
    printf("T1ActRobo-PosiSetByRTK end, x=%d,y=%d, myRoboM.fx=%f,myRoboM.fy=%f\n",x,y,myRoboM.fx,myRoboM.fy);
}

/* reset the meCmdCZ to control the rotation speed
 * 2026.05.04: created by Leo
 */
//void T1ActRobo::setCmdCZ(const int time, const char* cmdstr) {
void T1ActRobo::setSpeedRotate(const char cmd, const int val, const int time) {
    int debug=1;
    if (debug>5) printf("T1ActRobo-setSpeedRotate begin, time=%d, cmd=[%c,%d]\n", time, cmd,val);

    if ( val<45 ) return;
    if ( cmd!='C' && cmd!='c' && cmd!='Z' && cmd!='z' ) return;
    
    int speed=50;  // Be Careful!! speed is the rotation time per degree, not degree/second!!
    char cmdold = meCmdC;
    speed = time / val;
    if (debug>2) printf("T1ActRobo-setSpeedRotate step1, rollspeed=%d, time=%d, cmd=[%c,%d]\n", speed, time, cmd,val);
    if ( speed>50 ) {
        meCmdC='c'; meCmdZ='z';
    } else if ( speed<30 ) {
        meCmdC='C'; meCmdZ='Z';
    } else {
        ;
    }

    if (debug>0) printf("T1ActRobo-setSpeedRotate end, setCmd('%c'->'%c'), time=%d, cmd=[%c,%d], rollspeed=%d\n", cmdold, meCmdC, time, cmd,val, speed);
}

/* reset the meFact.SpeedWalk to control the walking speed
 * 2026.05.06: created by Leo
 * 2026.05.07: modified by Xs, add the Karlman filter gain to adjust the speed change, and log the process
 * 2026.05.25: modified by Leo, rewrrite the time judgement 
 */
void T1ActRobo::setSpeedWalk(PointRobo last) {
    int debug=1;
    char tmpstr[256];
    int flagset = FlagSetSpeed;
    if ( last.flagPosSetBy=='A' && myRoboM.flagPosSetBy=='A' ||
         last.flagPosSetBy=='R' && myRoboM.flagPosSetBy=='R' ) {
        ;
    } else {
        printf("T1ActRobo-setSpeedWalk failed step1, last=[%c], now=[%c]\n", last.flagPosSetBy, myRoboM.flagPosSetBy);
        return;
    }
    sprintf(tmpstr,"T1ActRobo-setSpeedWalk begin, last=[%c], now=[%c],myRoboM=(%.0f,%.0f,%d),last=(%.0f,%.0f,%d)", last.flagPosSetBy, myRoboM.flagPosSetBy, myRoboM.fx,myRoboM.fy,myRoboM.yaw, last.fx,last.fy,last.yaw);
    if (debug>1) T1Utils::LogRecord(tmpstr, "PosiSetByVisA",0);
    int dyaw = myRoboM.yaw - last.yaw;
    if ( abs(dyaw)>3 ) flagset=0;
    float dx = myRoboM.fx - last.fx;
    float dy = myRoboM.fy - last.fy;

    if ( last.timeSet==0) {
        printf("T1ActRobo-setSpeedWalk failed step2, last.timeSet=%ld\n", last.timeSet);
        return;
    }
    uint64_t dtime = myRoboVA.timeSet-last.timeSet;
    if ( dtime==0 || dtime>2000 ) {
        printf("T1ActRobo-setSpeedWalk failed step3, dtime=%ld\n", dtime);
        return;
    }

    float distance = sqrt( dx*dx + dy*dy );
    float speed = (distance/dtime*1000)*meFact.SpeedAdjustSpeed;
    int gainKarlman = getGainKarlman();

    float ds = speed-meFact.SpeedWalk1;
    sprintf(tmpstr,"T1ActRobo-setSpeedWalk step5, setBY=[%c], gainKar=%d, d-xydis=(%.1f,%.1f,%.1f),ds=%.1f, myRoboM=(%.0f,%.0f,%d)",
        last.flagPosSetBy, gainKarlman, dx,dy,distance,ds, myRoboM.fx,myRoboM.fy,myRoboM.yaw);
    if (debug>1) T1Utils::LogRecord(tmpstr, "PosiSetByVisA",1);
    if ( ds<-1*meFact.SpeedAdjustMax || ds>meFact.SpeedAdjustMax ) flagset=0;
    if ( ds<-1*meFact.SpeedAdjustMin) ds=-1*meFact.SpeedAdjustMin;
    if ( ds>meFact.SpeedAdjustMin ) ds=meFact.SpeedAdjustMin;
    float speedold = meFact.SpeedWalk1;
    if ( flagset ) {
        meFact.SpeedWalk1 += gainKarlman/10.0 * ds;
        meFact.SpeedWalk4 = meFact.SpeedWalk1;
    } else {
        printf("T1ActRobo-setSpeedWalk failed step4, flagset=%d, ds=%.1f\n", flagset, ds);
    }

    sprintf(tmpstr,"T1ActRobo-setSpeedWalk end, flag=%d, setBY=[%c], gainKarlman=%d, d-xydis=(%.1f,%.1f,%.1f), time=%ld, speed=(%.1f->%.1f), ds=%.1f->%d, myRoboM=(%.0f,%.0f,%d), last=(%.0f,%.0f,%d)",
        flagset, last.flagPosSetBy, gainKarlman, dx,dy,distance, dtime,speedold,speed, ds,meFact.SpeedWalk1, myRoboM.fx,myRoboM.fy,myRoboM.yaw, last.fx,last.fy,last.yaw);
    if (debug>0) T1Utils::LogRecord(tmpstr, "PosiSetByVisA",1);
    //meFact.SpeedWalk1 = speed;
    //meFact.SpeedWalk4 = meFact.SpeedWalk1;
}

/* get the gain of Karlman cause to spit time
 * 2023.05.06: created by Leo
 */
int T1ActRobo::getGainKarlman() {
    static uint64_t timelast = 0;
    uint64_t timenow, timediff;
    timenow = T1Utils::getCurrentTimeMs();
    timediff = timenow - timelast;
    timelast = timenow;

    int gainKarlman = 10;
    if ( timediff<meKarlmanS )
        gainKarlman = timediff*10/meKarlmanS + 1;

    return gainKarlman;
}

/* Change the Robot's position after one step act
 * 2023.05.06: created by Leo
 * 2026.05.18: modified by Leo, pass previous vision recognition time to setSpeedWalk for speed calculation
 * 2026.05.25: modified by Leo, rewrrite the time judgement and if (poxFixedVA==4)
 */
void T1ActRobo::RoboPosChange(char act, int val, int time) {
    //static uint64_t lastVisionTime = 0;
    char tmpstr[256];
    int debug=0;
    if (debug>3) printf("T1ActRobo-RoboPosChange begin. cmd=[%c,%d],time=%d\n", act,val,time);
    setSpeedRotate(act, val, time);
    if (debug>6) printf("T1ActRobo-RoboPosChange step1. setSpeedRotate successed.\n");
    // Save the current position before update
    PointRobo lastRoboM;
    lastRoboM.fx=myRoboM.fx; lastRoboM.fy=myRoboM.fy; lastRoboM.yaw=myRoboM.yaw;
    lastRoboM.flagPosSetBy = myRoboM.flagPosSetBy;
    lastRoboM.timeSet = myRoboM.timeSet;
    PosiSetByWheelO(act,val,time);
    if (debug>6) printf("T1ActRobo-RoboPosChange step3, set by wheel otometry. cmd=[%c,%d],time=%d\n", act,val,time);

    if ( mapBase.lon==-1 && mapBase.lat==-1 ) {
        myDiffMI.yaw = meDiffStation;    // maybe no need, already done when StationOut()
        if (debug>2) printf("T1ActRobo-RoboPosChange step4: myDiffMI.yaw set by Vision, meDiffStation=%d, myDiffMI.yaw=%d\n", meDiffStation, myDiffMI.yaw);
        if ( posFixedVA==4 ) {
            if ( act=='w' || act=='W' || act=='s' || act=='S' ) {
                if ( Tag2Robot() ) {
                    PosiSetByVisA();
                    //lastVisionTime = myRoboVA.timeSet;
                    posFixedVA = 0;
                }
            }
        }
    } else {
        myDiffMI.yaw = meDiffNorth;    // maybe no need, already done when StationOut()
        if (debug>2) printf("T1ActRobo-RoboPosChange step5: myDiffMI.yaw set by RTK, meDiffStation=%d, myDiffMI.yaw=%d\n", meDiffStation, myDiffMI.yaw);
        if ( posFixedRP==4 ) {
            PosiSetByRTK();
            posFixedRP = 0;
        }
    }
    /*if (myRoboVA.timeSet == lastVisionTime) {
         sprintf(tmpstr,"T1ActRobo-RoboPosChange step2, EARLY RETURN - wheel otometry skipped! cmd=[%c,%d],time=%d, lastVisionTime=%ld, myRoboVA.timeSet=%ld", act,val,time,myRoboVA.timeSet,lastVisionTime);
         if (debug>2) T1Utils::LogRecord(tmpstr, "timelock",1);
         //printf("T1ActRobo-RoboPosChange EARLY RETURN: Skipping PosiSetByWheelO!\n");
         return;
     }*/
    //myRoboM.flagPosSetBy = 'A'; // for test
    setSpeedWalk(lastRoboM);  // lastRoboM.timeSet contains the PREVIOUS vision recognition time for speed calculation
    
    // Update the last vision recognition time for next call
    // if (myRoboVA.timeSet > 0) {
    //     lastVisionTime = myRoboVA.timeSet;
    // } else {
    // }
    
    if (debug>3) printf("T1ActRobo-RoboPosChange end.\n");
}

/* Initializa the Robot's position before traversal
 */
void T1ActRobo::RoboPosInitial() {
    printf("T1ActRoboRobotPosInitial begin: ");
    if ( posFixedRP == 4 ) {
        ;
    } else {    // in room, or no satellite signal
        //myRoboM.fx = 0.0;//myBase.lon;  // ??? r u sure?
        //myRoboM.fy = 0.0;//myBase.lat;  // ??? r u sure?
        myRoboM.yaw = myRoboI.yaw;   // ??? r u sure? the rtk will give 0 after powerup indoor
        myDiffMI.yaw = myRoboM.yaw - myRoboI.yaw;
    }

    printf("T1ActRobo-RobotPosInitial end: %d, %d, %d\n", myRoboM.yaw, myRoboI.yaw, myDiffMI.yaw);
}

/* In one line of the map travarsal, Robot goto the target point step by step
 * return==0, successfully finished the line work,   return>0: steps worked in this line
 * return<0: failed in this line, but not used now
 */
int T1ActRobo::WorkLine(MapPoint target) {
    int debug=5;
    int ret;
    static char act='G';
    static int val;
    static char arcAct='Q';
    //if (debug) printf("\nT1ActRobo-WorkLine begin: line=%d,step=%d, target:(%lld,%lld,%d), RobotM:(%.1f,%.1f,%d), myDiffMI.yaw=%d\n", meWorkline, meWorkstep, target.lon, target.lat, target.yaw, myRoboM.fx, myRoboM.fy, myRoboM.yaw, myDiffMI.yaw);

    ret = 0;//RoboBangJudge();
    //if (debug) printf("T1ActRobo-WorkLine step1:" << meWorkline << meWorkstep << ret;
    //if (debug) printf("\n(%d,%d,%d)\n", meWorkline, meWorkstep, ret);
    if ( ret == 0 ) { // normal, safety
        ret = RoboAct('=',0);
        //if (debug) printf("T1ActRobo-WorkLine step1a: %d,%d, ret=%d\n", meWorkline, meWorkstep, ret);
    } else { //  will bang, danger
        /*val = ret;
        ret = RoboAct('G',0);
        RoboPosChange(0,act,ret);
        val = 0; // tempororay
        RoboAct('b',val);
        meWorkstep++;
        if (debug) printf("T1ActRobo-WorkLine step1b %d,%d, ret=%d\n", meWorkline, meWorkstep, ret);
        return meWorkstep;*/
    }

    //printf("T1ActRobo-WorkLine step2: %d,%d, ret=%d\n", meWorkline, meWorkstep, ret);
    if ( ret == 0 ) { // Acting......
        //printf("T1ActRobo-WorkLine step2a: %d,%d, ret=%d\n", meWorkline, meWorkstep, ret);
        ;
    } else if ( ret < 0 ) { // not act, prepare how to act
        // judge if reached the target
        MapPoint distance;
        distance.lon = target.lon - myRoboM.fx;
        distance.lat = target.lat - myRoboM.fy;
        //if ( abs(distance.lon)<STEPSAME && abs(distance.lat)<STEPSAME ) { // reached the target
        if ( abs(distance.lon)<meFact.StepCloseS && abs(distance.lat)<meFact.StepCloseS ) { // reached the target
            if (debug>5) printf("T1ActRobo-WorkLine step2b: (%d,%d), target:(%lld,%lld), distance:(%lld,%lld), myDiffMI.yaw=%d\n", meWorkline, meWorkstep, target.lon, target.lat, distance.lon, distance.lat, myDiffMI.yaw);
            act = 'g'; val=10; RoboAct(act,val);
            return 0;
        }
        if (debug>5) printf("T1ActRobo-WorkLine step3aa: (%d,%d), Act(%c,%d), myRoboM:(%f,%f,%d), Target:(%lld,%lld), IMU:%d, myDiffMI=%d\n",
           meWorkline, meWorkstep, act, val, myRoboM.fx, myRoboM.fy, myRoboM.yaw, target.lon, target.lat, myRoboI.yaw, myDiffMI.yaw) ;

        // judge how to act
        target.yaw = atan( (float)distance.lon / distance.lat )  * 180.0 / 3.14159265;
        if ( distance.lat < 0 ) target.yaw += 180;
        if (   target.yaw < 0 ) target.yaw += 360;
        if (debug>5) printf("T1ActRobo-WorkLine step3ab: target.yaw=%d ,distance:(%lld,%lld,%d), myDiffMI.yaw=%d\n",  target.yaw, distance.lon, distance.lat, distance.yaw, myDiffMI.yaw);
        distance.yaw = target.yaw - myRoboM.yaw;
        if ( distance.yaw < -180 ) distance.yaw += 360;
        else if ( distance.yaw > 180 ) distance.yaw -= 360;

        // if ( distance.yaw < NOTROT * (-1) ) { // must rotate//distance.yaw >45=C /distance.yaw <-45=Z
        if (distance.yaw < meFact.AngleRot * (-1) ) { // must rotate
            //act = 'z'; val = distance.yaw * (-1);
            act = meCmdZ; val = distance.yaw * (-1);
            if (debug>4) printf("T1ActRobo-WorkLine step3b-z: (%d,%d), Act(%c,%d), ", meWorkline, meWorkstep, act, val);
        } else if (distance.yaw > meFact.AngleRot ) { // must rotate
            //act = 'c'; val = distance.yaw;
            act = meCmdC; val = distance.yaw;
            if (debug>4) printf("T1ActRobo-WorkLine step3b-c: (%d,%d), Act(%c,%d), ", meWorkline, meWorkstep, act, val);
        } else {
            //val = meFact.StepLineg;
            val = meFact.StepLinec * 0.9;  // STEPLINE;
            int dist = sqrt ( distance.lon*distance.lon + distance.lat*distance.lat );
            if ( dist < meFact.StepCloseL ) {  // prevent jitter when close to the end
                act = 'w'; val = dist;
                if (debug>4) printf("T1ActRobo-WorkLine step3b-w: (%d,%d), Act(%c,%d), ", meWorkline, meWorkstep, act, val);
            } else {
                //if ( (-1)*NOTMODIFY<=distance.yaw && distance.yaw<=NOTMODIFY ) {  // walk ahead
                if ((-1)*meFact.AngleModi<=distance.yaw && distance.yaw<=meFact.AngleModi){ // walk ahead
                    act = 'W';
                    if (debug>4) printf("T1ActRobo-WorkLine step3b-W: (%d,%d), Act(%c,%d), ", meWorkline, meWorkstep, act, val);
                } else { // modify with walk
                    act = 'X'; val = distance.yaw;//val * RATEMODIFY;
                    //act = 'W';
                    if (debug>4) printf("T1ActRobo-WorkLine step3b-X: (%d,%d), Act(%c,%d), ", meWorkline, meWorkstep, act, val);
                }
            }
        }
        if (debug>2) printf("myRoboM:(%.0f,%.0f,%d), Target:(%.lld,%lld,%d), IMU:%d, agN-%d:%d, diatance:(%lld,%lld,%d)\n",
           myRoboM.fx, myRoboM.fy, myRoboM.yaw, target.lon, target.lat,target.yaw, myRoboI.yaw, meStageNorth, meAngleNorth, distance.lon, distance.lat, distance.yaw) ;
        RoboAct(act,val);
        meWorkstep++;
    } else { // ret>0, finished act, adjust the position after last act
        if (debug) printf("T1ActRobo-WorkLine step4a: %d,%d, Act(%c,%d), ret=%d, myRoboM:(%.0f,%.0f,%d), Target(%lld,%lld,%d), IMU:(%d), SpeedWalk1=%d\n", meWorkline,meWorkstep, act, val, ret
                 , myRoboM.fx, myRoboM.fy, myRoboM.yaw, target.lon, target.lat, target.yaw, myRoboI.yaw, meFact.SpeedWalk1);

        //if ( isAprilTagDetected() ) {
        //    posFixedVA = 4;
        //}

        RoboPosChange(act,val,ret);//val
        
        //if (debug) printf("T1ActRobo-WorkLine step4b: %d,%d, Act(%c,%d), ret=%d, myRoboM:(%lld,%lld,%d), Target(%lld,%lld,%d), IMU:(%d), Speed=%d\n", meWorkline,meWorkstep, act, val, ret
          //       , myRoboM.fx, myRoboM.fy, myRoboM.yaw, target.lon, target.lat, target.yaw, myRoboI.yaw, meFact.SpeedWalk1);
        if ( FlagStop == 8 ) {  // visual obstacle aviodence
            printf("T1ActRobo-WorkLine, FlagStop=%d\n", FlagStop);
            meWorkstep = 1000;
            FlagStop = 0;
        } else if ( FlagStop >= 5 || FlagBangRotating==1 ) {  // visual obstacle aviodence
            if ( FlagStop >= 5 && FlagBangRotating != 0 ) {
                if (debug) printf("T1ActRobo-WorkLine step4b again: %d,%d, FStop:%d, FRot:%d, redo obstacle avoidance\n",
                    meWorkline, meWorkstep, FlagStop, FlagBangRotating);
                FlagBangRotating = 0;
            }
            if ( FlagBangRotating == 0 ) {
                myRoboC.yaw = myRoboM.yaw;
                if ( FlagStop == 9 ) { //obstacle is on the right side
                    arcAct='E';
                    myRoboC.fx = myRoboM.fx + meFact.ArcleRadius * sin( myRoboM.yaw * 3.14159265 / 180 );
                    myRoboC.fy = myRoboM.fy + meFact.ArcleRadius * cos( myRoboM.yaw * 3.14159265 / 180 );
                    act='z';
                } else { //FlagStop==5,obstacle is on the left side
                    arcAct='Q'; 
                    myRoboC.fx = myRoboM.fx + meFact.ArcleRadius * sin( myRoboM.yaw * 3.14159265 / 180 );
                    myRoboC.fy = myRoboM.fy + meFact.ArcleRadius * cos( myRoboM.yaw * 3.14159265 / 180 );
                    act='c';
                }
                val=90;
                FlagBangRotating = 1;
                if (debug) printf("T1ActRobo-WorkLine step4b: %d,%d, FStop:%d, FRot:%d, Act(%c,%d), RoboM(%.1f,%.1f,%d), RoboC(%.1f,%.1f,%d)\n",
                    meWorkline, meWorkstep, FlagStop, FlagBangRotating, act, val, myRoboM.fx, myRoboM.fy, myRoboM.yaw, myRoboC.fx, myRoboC.fy, myRoboC.yaw);
                FlagStop = 0;
            } else {
                act=arcAct;
                val=meFact.ArcleDegree; //FlagStop+2;
                FlagBangRotating = 2;
                FlagStop = 0;
                if (debug) printf("T1ActRobo-WorkLine step4c: %d,%d, FStop:%d, FRot:%d, Act(%c,%d)\n", meWorkline, meWorkstep, FlagStop, FlagBangRotating, act, val);
            }
            RoboAct(act,val);
            if (debug) printf("T1ActRobo-WorkLine step4d:(%d,%d),FStop=%d, FRot=%d, act;[%c,%d]\n", meWorkline, meWorkstep, FlagStop, FlagBangRotating, act, val);
        }
    }
    //printf("T1ActRobo-WorkLine end: line=%d, step=%d\n", meWorkline, meWorkstep);
    //if (debug) printf("T1ActRobo-WorkLine end.");
    return meWorkstep;
}

/* Robot begin to traversal the map
 * return==1: working,  return<0: finished work,
 */
int T1ActRobo::Work() {
    //printf("T1ActRobo-Work() begin: WS2=%d, WSR=%d, (%d,%d,%d), Min:(%lld,%lld), Max:(%lld,%lld)\n", WorkStatus2, WorkStatusR, meWorklength, meWorkline, meWorkstep, mapMin.lon, mapMin.lat, mapMax.lon, mapMax.lat);
    if ( WorkStatusR < 0 || FlagPause == 1) { // Stopped work
        ;
    } else if ( WorkStatusR == 0 ) { // Initialize to work
        printf("\n--------- T1ActRobo-Work, WSR=0 ---------\n");
        //RoboAct('0',0); // reset all action
        RoboAct('I',195);
        pMap->SetRouSet(meFact.StepLinex,meFact.StepLiney);
        pMap->GetRoute(0); mapBase = pMap->GetPoint('b',0); mapMin = pMap->GetPoint('i',0); mapMax = pMap->GetPoint('a',0);
        meWorklength = pMap->GetLength('P'); meWorkline = 0; meWorkstep = 0;
        //RoboPosInitial();
        WorkStatusR = 1;
        printf("T1ActRobo-Work() step1: (%d,%d,%d), WSR=%d, Min:(%lld,%lld), Max:(%lld,%lld), RoboM:(%.1f,%.1f)\n", meWorklength, meWorkline, meWorkstep, WorkStatusR, mapMin.lon, mapMin.lat, mapMax.lon, mapMax.lat, myRoboM.fx, myRoboM.fy);
    } else { // WorkStatus==1: working
        //printf("T1ActRobo-Work() step2: %ld, RSW=%d, %d, %d, Min:(%lld,%lld), Max:(%lld,%lld)\n", myRoutePlan.size(), WorkStatusR, meWorkline, meWorkstep, mapMin.lon, mapMin.lat, mapMax.lon, mapMax.lat);
        if ( meWorkline >= meWorklength ) { // all line worked
            WorkStatusR = -1;
            //WorkStatusR = 0;
            printf("T1ActRobo-Work() step3, scaned all lines: %d, RSW=%d, meWorkline=%d, %d\n\n", meWorklength, WorkStatusR, meWorkline, meWorkstep);
            return WorkStatusR;
        }
        int ret = WorkLine(pMap->GetPoint('P', meWorkline));
        if ( ret == 0 ) {
            if ( meWorkline == 0 )
                printf("T1ActRobo-Work() step2a, scaned first lines: (%d,%d,%d), WSR=%d, Base:(%lld,%lld)\n", meWorklength, meWorkline, meWorkstep, WorkStatusR, mapBase.lon, mapBase.lat);
            else if ( meWorkline >= meWorklength-1 )
                printf("T1ActRobo-Work() step2c, scaned last lines: (%d,%d,%d), WSR=%d\n", meWorklength, meWorkline, meWorkstep, WorkStatusR);
            else
                printf("T1ActRobo-Work() step2b, scaned one lines: (%d,%d,%d), WSR=%d, NextRoute:(%lld,%lld)\n", meWorklength, meWorkline, meWorkstep, WorkStatusR, pMap->GetPoint('P',meWorkline+1).lon, pMap->GetPoint('P',meWorkline+1).lat);
            meWorkline ++;
            meWorkstep = 0;
        }
    }
    //printf("T1ActRobo-Work() end: %ld, RSW=%d, %d, %d, Min:(%lld,%lld), Max:(%lld,%lld)\n", meWorklength, WorkStatusR, meWorkline, meWorkstep, mapMin.lon, mapMin.lat, mapMax.lon, mapMax.lat);
    return WorkStatusR;
}

/* return: true, detected a AprilTag,    false: nothing
 * 2026.05.01: created by Leo
 * 2026.05.02: modified by Leo, use new function to calculate the camera's position
 */
bool T1ActRobo::Tag2Robot() {
    char debugstr[256];
    int debug=1;
    if (debug>5) printf("T1ActRobo-Tag2Robot begin, myRoboM(%.1f,%.1f,%d)\n", myRoboM.fx, myRoboM.fy, myRoboM.yaw);
    float visionX = myRoboVA.fx;
    float visionY = myRoboVA.fy;
    int visionYaw = myRoboVA.yaw;
    uint64_t visionTime = myRoboVA.timeSet;

    // Discard vision data older than 1 second
    uint64_t timeNow = T1Utils::getCurrentTimeMs();
    if ( visionTime == 0 || (timeNow - visionTime) > 1000 ) {
        if (debug>1) printf("T1ActRobo-Tag2Robot end, vision data too old: visionTime=%ld, timeNow=%ld, diff=%ldms\n",
                           visionTime, timeNow, timeNow - visionTime);
        return false;
    }

    if (debug>2) printf(">>>T1ActRobo-Tag2Robot step1, Using T1All vision data: (%.1f,%.1f,%d), time=%ld, age=%ldms\n",
                       visionX, visionY, visionYaw, visionTime, timeNow - visionTime);
    if ( visionYaw>meFact.ATagRangeA || visionYaw<-1*meFact.ATagRangeA ) {
        if (debug>1) printf("T1ActRobo-Tag2Robot end, visionYaw=%d out of range\n", visionYaw);
        return false;
    }
    int length = ((T1Map2 *)pMap)->myMapAprilTag.size();
    for ( int i=0; i<length; i++ ) {
        MapPoint tmppoint = ((T1Map2 *)pMap)->GetPoint2('A',i);
        if (debug>3) printf("T1ActRobo-Tag2Robot loop-step1(%d), myRoboM.yaw=%d, TagSet=(%lld,%lld,%d)\n", i, myRoboM.yaw, tmppoint.lon, tmppoint.lat, tmppoint.yaw);
        if ( !isAprilTagInView(tmppoint.lon,tmppoint.lat) ) continue;
        if (debug>3) printf("T1ActRobo-Tag2Robot loop-step2(%d), myRoboM.yaw=%d, TagSet=(%lld,%lld,%d)\n", i, myRoboM.yaw, tmppoint.lon, tmppoint.lat, tmppoint.yaw);
        // Use vision data from myRoboVA (from T1All) and calculate camera position
        myPosVisA = Tag2Camera(visionX, visionY, myRoboM.yaw, tmppoint.lon, tmppoint.lat);
        
        sprintf(debugstr, "T1ActRobo-Tag2Robot successed: TagSetupPos=(%d,%lld,%lld), VisionData=(%.1f,%.1f,%d), VisPos=(%.1f,%.1f), RobotPos=(%.1f,%.1f,%d)",
            i, tmppoint.lon, tmppoint.lat, visionX, visionY, visionYaw, myPosVisA.fx, myPosVisA.fy, myRoboM.fx, myRoboM.fy, myRoboM.yaw);
        if (debug>0) T1Utils::LogRecord(debugstr, "PosiSetByVisA",1);
        // T1Utils::LogBug(debugstr);
        
        return true;
    }
    if (debug>1) printf("T1ActRobo-Tag2Robot end, myRoboM.yaw=%d\n", myRoboM.yaw);
    return false;
}

/** Check if AprilTag is in the mower's field of view
 * Return: true = tag is visible (in range + in angle), false = not visible
 * 2026.05.03: created by Leo
 */
bool T1ActRobo::isAprilTagInView(long long tagSetupX, long long tagSetupY)
{
    int debug=0;
    
    // 1. Calculate relative distance between robot and tag
    float dx = tagSetupX - myRoboM.fx;
    float dy = tagSetupY - myRoboM.fy;
    float distance = sqrt(dx * dx + dy * dy);
    if (debug>3) printf("T1ActRobo-isAprilTagInView begin, TagSet(%lld,%lld), myRoboM(%.0f,%.0f), dxy(%.2f,%.2f), distance=%.0f\n",
        tagSetupX,tagSetupY, myRoboM.fx,myRoboM.fy, dx,dy, distance);

    // Check distance first: tag must be within allowed range
    if (distance > meFact.ATagRangeD)
        return false;

    // 2. Calculate the angle from ROBOT to TAG (convert to DEGREES: -180° ~ 180°)
    float angleToTag = atan2(dx, dy) * 180.0f / M_PI;
    if (debug>2) printf("T1ActRobo-isAprilTagInView step1, angleToTag=%.0f\n", angleToTag);

    // 3. Get robot's current yaw (facing direction)
    float robotYaw = myRoboM.yaw;

    // 4. Calculate the robot's view window (left/right angle limit)
    int halfViewAngle = meFact.ATagRangeA / 2;
    float angleMin = robotYaw - halfViewAngle;
    float angleMax = robotYaw + halfViewAngle;

    // 5. Normalize all angles to 0° ~ 360° to avoid wrap bugs
    angleToTag = fmod(angleToTag + 360.0f, 360.0f);
    angleMin = fmod(angleMin + 360.0f, 360.0f);
    angleMax = fmod(angleMax + 360.0f, 360.0f);

    if (debug>1) printf("T1ActRobo-isAprilTagInView step3, angleToTag=%.0f, range(%.0f,%.0f)\n", angleToTag, angleMin,angleMax);
    // 6. Check if tag is inside the angle range
    bool inAngle;
    if ( angleMin<=angleMax ) {
        inAngle = ( angleToTag>=angleMin ) && ( angleToTag<=angleMax );
    } else {
        // Handle 0°/360° boundary crossing
        inAngle = ( angleToTag>=angleMin ) || ( angleToTag<=angleMax );
    }

    if ( inAngle ) {
        if (debug>0) printf("T1ActRobo-isAprilTagInView successed, dxy(%.0f,%.0f), angleToTag=%.0f, range(%.0f,%.0f)\n", dx,dy, angleToTag, angleMin,angleMax);
    }
    // Return final result: both distance AND angle are valid
    return inAngle;
}

/* calculate the camera's position from the AprilTag's position
 * 2026.05.02: created by Leo
 * 2026.05.19: modifyed by Leo, change the return type
 */
PointRobo T1ActRobo::Tag2Camera(float tagX, float tagY, int roboYaw, long long tagSetX, long long tagSetY) {
    const double DEG2RAD = M_PI / 180.0;
    PointRobo camera;
    //printf("T1ActRobo-Tag2Camera begin, tagPosXY=(%.1f,%.1f), roboYaw=%d, tagSetXY=(%lld,%lld)\n", tagX,tagY,roboYaw,tagSetX,tagSetY);
    float sinYaw = sin(roboYaw*DEG2RAD);
    float cosYaw = cos(roboYaw*DEG2RAD);
    camera.fx = tagSetX - tagY*sinYaw - tagX*cosYaw;
    camera.fy = tagSetY - tagY*cosYaw + tagX*sinYaw;
    //printf("T1ActRobo-Tag2Camera end, YawSinCos=(%d,%.3f,%.3f), TagXY(%.1f,%.1f)->cameraXY(%.1f,%.1f)\n", roboYaw, sinYaw, cosYaw, tagX, tagY, camera.fx, camera.fy);
    return camera;
}
