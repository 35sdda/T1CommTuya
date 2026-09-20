#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>

#include "T1Firm.hpp"
#include "T1Utils.hpp"

T1Firm::T1Firm()
{

}

T1Firm::~T1Firm()
{
}

/* process analysis datas from 415, return:
 *  = 4: STA success, $STA (status/sensor data)
 *  = 3: stop success, $stop
 *  = 2: IMU success, $roll
 *  = 1: RTK success, $GNGGP/$GPGGA
 *  = 0: do nothing return, $command
 *  =-1: will reboot 415 in TheOne.cpp::main()
 *  =-2: need to send factory parameters to 415
 *  <-10: error
 * 2026.05.16: moved from T1Act.cpp by Leo
 * 2026.05.19: modified return define by Leo
 * 2026.06.02: add return -10, by Leo
 * 2026.06.02: add return 4, by Xs
 */
int T1Firm::recvLon( const char * data) {
    int ret = -10;

    static int count=0;
    char tmpstr[300];

    static uint64_t timepowerup=0, timenow=0, timeGNGGA=0, timeRoll=0, timeSTA=0;
    timenow = T1Utils::getCurrentTimeMs();
    if ( timepowerup == 0 ) timepowerup=timenow; // the time when The1 start up

    //LogLonDelta(data, "STM415");
    //printf("================T1Firm-recvLon begin: [%s]================\r\n", data);
    std::string str(data);
    std::vector<std::string> list1;
    list1 = T1Utils::split(str,',');
    size_t l1length = list1.size(); 
    //printf("T1Firm-recvLon begin: %lu,[%s]\r\n", l1length, str.c_str()); //list1[0].c_str());
    if ( list1[0] == "$Info" ) {
        //FlagInfomation = FlagInfomation | 0x80;
        //printf("T1Firm-recvLon $Info: %lu,[%s]\n", l1length, data);//list1[0].c_str());
    } else if ( list1[0][1] == 'G' ) {  // GNGGA, GPGGA
    //} else if ( list1[0] == "$GNGGA" ) {
        //printf("T1Firm-recvLon $GNGGA: %lu,[%s]\n", l1length, data);//list1[0].c_str());
        timeGNGGA = timenow;
        //if (meFact.RtkSetFlag == 0) {
        //    ;} else 
        if ( l1length == 15 || l1length==14 ) { // 14 means no base RTK's signal
    //15,"$GNGGA,20240416063817.11,120.38758450,31.52241876,16.4509,356.13,-0.61,356.13,0.008,-0.07,5,0,18,18,-0.154,-0.164,-0.016,-0.004,0.006,0.000,10,0*4B\r"
    //14,"$GNGGA,081817.00,3134.70257780,N,12020.97268343,E,1,28,0.6,32.0376,M,7.4475,M,,*75"
            ret = processGNGGA(list1,timenow);
        } else {
            //posFixedRP = -1;
            //$command,GNGGA,,,,,,0,,,,,,,,,response: PARSING FAILD GRAMMAR ERROR,,,,,0,,,,,,,,*17
            //T1All-ProcessFirmSerial, recvLon error for line: $command,GNGGA,,,,,,0,,,,,,,,,respons
            ret = -10;
            sprintf(tmpstr,"T1Firm-recvLon $GNGGA-x error:%ld,[%s]\n", l1length, data);
            LogLonBug(tmpstr);
        }
    } else if ( list1[0] == "$IMU" ) {
        timeRoll = timenow;
        // printf("T1Firm-recvLon $IMU: %lu,[%s]\n", l1length, data);
        if ( l1length==8 ) {// "$IMU,0.008,0.138,0.412,-0.012,0.004,1.006,124678"
            ret=processIMU(list1, timenow);
            sprintf(tmpstr,"T1Firm-recvLon $IMU-x success,[%s]\n", data);
            T1Utils::LogRecord(tmpstr, "415IMU");
        } else {
            ret = -10;
            sprintf(tmpstr,"T1Firm-recvLon $IMU-x error,[%s]", data);
            T1Utils::LogBug(tmpstr);
            //printf("T1Firm-recvLon $IMU: %lu,[%s]\r\n", l1length, list1[0].c_str());
        }
    } else if ( list1[0] == "$roll" ) {
        //printf("T1Firm-recvLon $roll: %lu,[%s]\n", l1length, data);
        timeRoll = timenow;
        if ( l1length==5 ) {// "$roll,-0.07,pitch,-0.61,yaw,356.13,count,1234546,x,y,yaw,baselon,baselat\r"
            ret = processRoll(list1,timenow);
            sprintf(tmpstr,"T1Firm-recvLon $roll-x success,[%s]\n", data);
            T1Utils::LogRecord(tmpstr, "415roll");
        } else {
            ret = -10;
            sprintf(tmpstr,"T1Firm-recvLon $roll-x error,[%s]", data);
            T1Utils::LogBug(tmpstr);
        }
    } else if ( list1[0] == "$stop" ) {
        if ( l1length == 4 ) {  // "$stop,[RotR],yaw,[20.016621]"
            ret = processStop(list1, timenow);
            printf("T1Firm-recvLon $stop, processStop ret=%d\n", ret);
        } else {
            ret = -10;
            sprintf(tmpstr,"T1Firm-processStop stop error: %ld[%s]\n", list1.size(), data);
            T1Utils::LogBug(tmpstr);
        }
    } else if ( list1[0] == "$STA" ) {
    // $STA,ver=0.3.1,bat=12.45,L=0,R=0,C=0,RL=0,RR=0,S1=0,S2=0,Rain=0
    // $STA,0.3.1,12.45,0,0,0,0,0,0,0,0
        timeSTA = timenow;
        if ( l1length==13 ) {
            ret = processSTA(list1,timenow);
        } else {
            ret = -10;
            sprintf(tmpstr,"T1Firm-recvLon $STA error,%ld:[%s]", l1length, data);
            T1Utils::LogBug(tmpstr);
        }
    } else if ( list1[0] == "$CMD" ) {
        ret = processCMD(list1,timenow);
    } else if ( list1[0] == "$command" ) {
        //$command,GNGGA,,,,,,0,,,,,,,,,response: PARSING FAILD GRAMMAR ERROR,,,,,0,,,,,,,,*17
        sprintf(tmpstr,"T1Firm-recvLon in-room RTK: %ld,[%s]", l1length, data);
        LogLonBug(tmpstr);
        ret = 0;
    } else if ( list1[0] == "$factory" ) { //
        sprintf(tmpstr,"T1Firm-recvLon need factory parameters.");
        T1Utils::LogBug(tmpstr);
        ret = -2;
    } else {
        sprintf(tmpstr,"T1Firm-recvLon unknown: %ld,[%s]\n", l1length, data);
        T1Utils::LogBug(tmpstr);
        ret = -10;
    }
    
    if ( timenow-timepowerup > 5*meTimeout415 ) {
        //printf("T1Firm-recvLon timeout a: %ld,%ld,%ld,%ld\n", timenow, timeRoll, timeGNGGA, timeSTA);
        //if ( FlagReset415 == 1 ) FlagPause = 0;
        if ( timenow-timeGNGGA > meTimeout415 ) {
            if ( FlagRtkReset!=0 ) {
                sprintf(tmpstr,"T1Firm-recvLon GNGGA timeout, timenow=%ld, timeGNGGA=%ld, diff=%ld", timenow, timeGNGGA, timenow-timeGNGGA);
                T1Utils::LogBug(tmpstr);
                timepowerup = timenow;
                ret = -1; // will reboot 415 in TheOne.cpp
            }
        }
        if ( timenow-timeRoll > meTimeout415 ) {
            sprintf(tmpstr,"T1Firm-recvLon Roll timeout, timenow=%ld, timeRoll=%ld, diff=%ld", timenow, timeRoll, timenow-timeRoll);
            T1Utils::LogBug(tmpstr);
            timepowerup = timenow;
            ret = -1; // will reboot 415 in T1All.cpp
            //reset415();
        }
        /*if ( timenow-timeSTA > meTimeout415 ) {
            sprintf(tmpstr,"STA timeout, timenow=%ld, timeSTA=%ld, diff=%ld", timenow, timeSTA, timenow-timeSTA);
            LogBug(tmpstr);
            timepowerup = timenow;
            printf("T1Firm-recvLon timeout: %s\n", tmpstr);
            // should tell 415 to reboot sensors
        }*/
        //printf("T1Firm-recvLon timeout b: %ld,%ld,%ld,%ld\n", timenow, timeRoll, timeGNGGA, timeSTA);
    }

    //if ( count%10==0 ) std::cout << "T1Firm-recvLon step4c: " << posFixedRY << myMower5.roll << myMower5.pitch << myMower5.yaw << posFixedRP << myMower5.lon << myMower5.lat << myMower5.alt;
    count ++;
    return ret;
}

/* process the "$GNGGA" string for the 415
 * return: -10 if error， >0 if success
 * 2026.05.17: creat by Leo
 */ 
int T1Firm::processGNGGA(const std::vector<std::string> list1, uint64_t timenow) {
    char tmpstr[300];
    int ret = -10;
    double utime=0,lon=0,lat=0,alt=0;//,roll=0,pitch=0,yaw=0;
    int sateA=0;//, sateB=0;

    //15,"$GNGGA,20240416063817.11,120.38758450,31.52241876,16.4509,356.13,-0.61,356.13,0.008,-0.07,5,0,18,18,-0.154,-0.164,-0.016,-0.004,0.006,0.000,10,0*4B\r"
    //14,"$GNGGA,081817.00,3134.70257780,N,12020.97268343,E,1,28,0.6,32.0376,M,7.4475,M,,*75"
    //printf("T1Firm-recvLon processGNGGA: %ld,[%s]\n", l1length, data);

    if ( 1 != sscanf(list1[1].c_str(),"%lf",&utime) ) {
        ret = -10;
        printf("T1Firm-recvLon sscanf error.\n");
        return ret;
    }
    try {
        myPosRTK.timeSet = std::stod(list1[1]);
        lat = std::stod(list1[2]);
        if ( list1[3][0]=='S' ) lat *= -1;
        lon = std::stod(list1[4]);
        if ( list1[5][0]=='W' ) lon *= -1;
        myPosRTK.fixed = std::stoi(list1[6]);
        myPosRTK.sate = std::stoi(list1[7]);
        myPosRTK.alt = std::stod(list1[9]) * 100;
        //int posFixedRY = std::stoi(list1[11]);
        //int posFixedRP = std::stoi(list1[7]);
        //int StataNum = myPosRTK.sate;
        //sateB = list1[13].toInt();
        // rtkFixedPolaris = posFixedRP;
        ret = 1;
    } catch (const std::invalid_argument& e) {
        sprintf(tmpstr,"T1Firm-processGNGGA$ error argumetn, [%s]",  e.what());
        LogLonBug(tmpstr);
        ret = -10;
    } catch (const std::out_of_range& e) {
        sprintf(tmpstr,"T1Firm-processGNGGA$ error out of range, [%s]",  e.what());
        LogLonBug(tmpstr);
        ret = -10;
    }
    //if ( posFixedDemo != -1 ) posFixedRP = posFixedDemo;
    myPosRTK.lon = T1Utils::TransLonLat(lon) * PRECISION;
    myPosRTK.lat = T1Utils::TransLonLat(lat) * PRECISION;
    // somethig error when lon/lat is 0
    if ( lon < -180*PRECISION || lon > 180*PRECISION ) {
        sprintf(tmpstr,"T1Firm-processGNGGA$ error longitude out of range, [%s,%s,%s,%s,%s,%s,%s,%s,%s]", list1[1].c_str(), list1[2].c_str(), list1[3].c_str(), list1[4].c_str(), list1[5].c_str(), list1[6].c_str(), list1[7].c_str(), list1[8].c_str(), list1[9].c_str());
        LogLonBug(tmpstr);
        ret = -10;
        return ret;
    }
    if ( lat < -90*PRECISION || lat > 90*PRECISION ) { //"$GNGGA,,,,,,0,,,,,,,,*78\r"
        sprintf(tmpstr,"T1Firm-processGNGGA$ error latitude out of range, [%s,%s,%s,%s,%s,%s,%s,%s,%s]", list1[1].c_str(), list1[2].c_str(), list1[3].c_str(), list1[4].c_str(), list1[5].c_str(), list1[6].c_str(), list1[7].c_str(), list1[8].c_str(), list1[9].c_str());
        LogLonBug(tmpstr);
        ret = -10;
        return ret;
    }
    // if ( posFixedRP == 4 ) {
    //     //myRTKF.lon = meScaleLon * ( myRTK.lon - mapBase.lon );
    //     //myRTKF.lat = meScaleLat * ( myRTK.lat - mapBase.lat );
    //     myPosRTKF.lon = 1 * ( myPosRTK.lon - mapBase.lon );
    //     myPosRTKF.lat = 1 * ( myPosRTK.lat - mapBase.lat );
    //     myPosRTKF.alt = myPosRTK.alt;
    // }
    // if ( WorkStatusT >= 0 )
    //    MapByTrail(data);
    
    //FlagInfomation = FlagInfomation | 0x02;
    //if ( count%10 == 0 ) std::cout << "T1Firm-recvLon $GNGGA ok: " << list1.size() << line;
    //if ( FlagTestNight==1 && count%5==0 ) 
        //printf("T1Firm-recvLon ok: %ld,[%s]\n", list1.size(), data);// << "IMU:" << myRoboI.roll << myRoboI.pitch << myRoboI.yaw << "Robo:" << myRoboM.yaw << myDiffMI.yaw;

    return 1;
}

/* process the "$roll" string for the 415
 * return: -10 if error， 2 if success
 * 2026.05.17: creat by Leo
 */ 
int T1Firm::processRoll(const std::vector<std::string> list1, uint64_t timenow) {
    //double utime=0,lon=0,lat=0,alt=0,roll=0,pitch=0,yaw=0;
    //printf("T1Firm-processRoll $roll=[%s,%s,%s,%s,%s,%s,%s,%s,%s]", list1[1].c_str(), list1[2].c_str(), list1[3].c_str(), list1[4].c_str(), list1[5].c_str(), list1[6].c_str(), list1[7].c_str(), list1[8].c_str(), list1[9].c_str());
    try {
        int roll = std::stod(list1[1]) * 100;
        int pitch = std::stod(list1[2]) * 100;
        int yaw = std::stod(list1[3]) * 100;
        int imucount = std::stoi(list1[4]);

        myRoboI.timeSet = timenow;
        myRoboI.alt = imucount;  // used to carry the runtime of 415

        myRoboI.roll = roll/100;
        myRoboI.pitch = pitch/100;
        myRoboI.yaw =  -yaw/100; // -180~180 -> 0 to 360
        if ( myRoboI.yaw < 0 ) myRoboI.yaw += 360;
        // myRoboM.yaw = myRoboI.yaw + myDiffMI.yaw;
        // if ( myRoboM.yaw >= 360 ) myRoboM.yaw -= 360;
        // if ( myRoboM.yaw < 0 ) myRoboM.yaw += 360;
        return 2; // success
        
    
        //LogLonBug(data);
    } catch (const std::exception &e) {
        char tmpstr[300];
        sprintf(tmpstr,"T1Firm-processRoll $roll parse failed, [%s,%s,%s,%s,%s,%s,%s,%s,%s]", list1[1].c_str(), list1[2].c_str(), list1[3].c_str(), list1[4].c_str(), list1[5].c_str(), list1[6].c_str(), list1[7].c_str(), list1[8].c_str(), list1[9].c_str());
        T1Utils::LogBug(tmpstr);
        return -10;
    }

    //FlagInfomation = FlagInfomation | 0x01;
    //myMower6.yaw = myMower5.yaw - mapBase.yaw;
    // if (TestNight==1 && count%5==0 ) printf("T1Firm-processRoll ok: %ld,[%s]\n", list1.size(), data);// << "IMU:" << myRoboI.roll << myRoboI.pitch << myRoboI.yaw << "Robo:" << myRoboM.yaw << myDiffMI.yaw;
    //std::cout << "T1Firm-processRoll $roll:"<< list1.size() << line ;

    return -2;
}
/* process the "$IMU" string for the 415
 * return: -2 if error， 2 if success
 * 2026.06.04: creat by Xs
 */ 
int T1Firm::processIMU(const std::vector<std::string> list1, uint64_t timenow) {
    //$IMU,0.008,0.138,0.412,-0.012,0.004,1.006,124678
    try {
        myRoboI.gyroX=std::stof(list1[1]);
        myRoboI.gyroY=std::stof(list1[2]);
        myRoboI.gyroZ=std::stof(list1[3]);
        myRoboI.accX=std::stof(list1[4]);
        myRoboI.accY=std::stof(list1[5]);
        myRoboI.accZ=std::stof(list1[6]);
        myRoboI.timeIMU415=std::stoul(list1[7]);
        myRoboI.timeIMU=timenow;
        return 5;
    } catch ( const std::exception &e ) {
        char tmpstr[300];
        sprintf(tmpstr,"T1Firm-processIMU $IMU parse failed, [%s,%s,%s,%s,%s,%s,%s]", list1[1].c_str(), list1[2].c_str(), list1[3].c_str(), list1[4].c_str(), list1[5].c_str(), list1[6].c_str(), list1[7].c_str());
        T1Utils::LogBug(tmpstr);
        return -10;
    }
}

/* process the "$stop" string for the 415
 * return: -10 if error， >0 if success
 * 2026.05.17: creat by Leo
 */ 
int T1Firm::processStop(const std::vector<std::string> list1, uint64_t timenow) {
    int ret = -10;
    if ( list1[1] == "[RotL-Z]" ) {
        myRoboI.sate = 1;  // used to carry the FlagStop of RotL-Z
        ret = 3;
        //FlagStop = 1;
    } else if ( list1[1] == "[RotR-C]" ) {
        myRoboI.sate = 2;  // used to carry the FlagStop of RotR-C
        ret = 3;
    } else if ( list1[1] == "[ModiL X+]" ) {
        myRoboI.sate = 3;  // used to carry the FlagStop of ModiL X+
        ret = 3;
    } else if ( list1[1] == "[ModiR X-]" ) {
        myRoboI.sate = 4;  // used to carry the FlagStop of ModiR X-
        ret = 3;
    } else if ( list1[1] == "[ActObs Q]" ) {
        myRoboI.sate = 6;  // used to carry the FlagStop of ActObs Q
        ret = 3;
    } else if ( list1[1] == "[ActObs E]" ) {
        myRoboI.sate = 7;  // used to carry the FlagStop of ActObs E
        ret = 3;
    } else {
        printf("T1Firm-processStop stop unknown: %s\n", list1[1].c_str());
        ret = -10;
    }
    myRoboI.timeSet = timenow;

    int yaw = std::stod(list1[3].substr(1)) * 100;
    myRoboI.yaw =  -yaw/100; // -180~180 -> 0 to 360
    if ( myRoboI.yaw<0 ) myRoboI.yaw += 360;
    
    //myRoboM.yaw = myRoboI.yaw + myDiffMI.yaw;
    //while ( myRoboM.yaw >= 360 ) myRoboM.yaw -= 360;
    //while ( myRoboM.yaw < 0 ) myRoboM.yaw += 360;
    //printf("T1Firm-processStop stop ok: %ld[--], FlagStop=%d, myRoboM.yaw=%d\n", list1.size(), FlagStop, myRoboM.yaw);            

    return ret;
}
/* process the "$STA" string from the 415
 * return: -10 if error， >0 if success
 * 2026.05.17: creat by Leo
 * 2026.06.02: modified to save values to STAStatus struct
 */
int T1Firm::processSTA(const std::vector<std::string> list1, uint64_t timenow) {
    // $STA,ver=0.3.1,bat=12.45,L=0,R=0,C=0,RL=0,RR=0,S1=0,S2=0,Rain=0
    // $STA,0.3.1,12.45,0,0,0,0,0,0,0,0
    char tmpstr[300];
    size_t l1length = list1.size();

    if ( l1length >= 13 ) {
        try {
            std::string ver = list1[1];   // "0.3.1"
            std::string bat = list1[2];   // "12.45"
            int cflag = std::stoi(list1[3]);   // Charging marker position
            int sL    = std::stoi(list1[4]);   // sen_front_left
            int sR    = std::stoi(list1[5]);   // sen_front_right
            int sC    = std::stoi(list1[6]);   // sen_center
            int sRL   = std::stoi(list1[7]);   // sen_rear_left
            int sRR   = std::stoi(list1[8]);   // sen_rear_right
            int sS1   = std::stoi(list1[9]);   // sen_stop1
            int sS2   = std::stoi(list1[10]);  // sen_stop2
            int sRain = std::stoi(list1[11]);  // sen_rain
            int gHome = std::stoi(list1[12]);  // Go_home
            //const double parsedVoltage = std::stod(bat);

            // Save all parsed values to STAStatus struct
            strncpy(mySTAStatus.version, ver.c_str(), sizeof(mySTAStatus.version) - 1);
            mySTAStatus.version[sizeof(mySTAStatus.version) - 1] = '\0';
            mySTAStatus.batteryVoltage = std::stod(bat);
            mySTAStatus.chargeFlag = cflag;
            mySTAStatus.senFrontLeft = sL;
            mySTAStatus.senFrontRight = sR;
            mySTAStatus.senCenter = sC;
            mySTAStatus.senRearLeft = sRL;
            mySTAStatus.senRearRight = sRR;
            mySTAStatus.senStop1 = sS1;
            mySTAStatus.senStop2 = sS2;
            mySTAStatus.senRain = sRain;
            mySTAStatus.goHome = gHome;
            // mySTAStatus.timeSet = timenow;

            // // Apply exponential moving average for battery voltage
            // const double alpha = 0.6;
            // if (mySTAStatus.batteryVoltage <= 0.0) {
            //     mySTAStatus.batteryVoltage = parsedVoltage;  // first sample
            // } else {
            //     mySTAStatus.batteryVoltage = alpha * parsedVoltage + (1.0 - alpha) * mySTAStatus.batteryVoltage;
            // }

            return 4;  // return 4 for $STA success, will trigger sync in T1All
        } catch (const std::exception &e) {
            sprintf(tmpstr,"T1Firm-processSTA $STA parse failed, exception: %s", e.what());
            T1Utils::LogBug(tmpstr);
            return -10;
        }
    } else {
        sprintf(tmpstr,"T1Firm-processSTA $STA length failed, expected>=13, got=%zu", l1length);
        T1Utils::LogBug(tmpstr);
        return -10;
    }
}

/* process the "$CMD" string for the 415
 * return: -10 if error， >0 if success
 * 2026.05.17: creat by Leo
 */ 
int T1Firm::processCMD(const std::vector<std::string> list1, uint64_t timenow) {
    //printf("T1Firm-recvLon $CMD step1: %ld,[%s]\n", list1.size(), data);
/*
    if ( l1length == 4 ) {  // "$CMD,ok,T=10,100*E6"
        if ( list1[2][0] == 'T' ) {// "$CMD,ok,T=10,100*E6"
            if ( list1[1] == "ok" ) {  // "$CMD,ok,T=10,100*E6"
                printf("T1Firm-recvLon $CMD step-a: %d, [%s]\n", meWorkstep, data);// << yaw/100;// << line;
                meWorkstep = 2;  // target received
            } else if ( list1[1] == "reach" ) { // "$CMD,reach,T=10,100*E6"
                if ( meWorkstep == 2 ) {
                    printf("T1Firm-recvLon $CMD step-b: %d, [%s]\n", meWorkstep, data);// << yaw/100;// << line;
                    meWorkstep = 3;  // target reached
                }
            } else {  // "$CMD,error,T=10*E6"
                printf("T1Firm-recvLon $CMD step-c: %d, [%s]\n", meWorkstep, data);// << yaw/100;// << line;
            }
        }
    } else if ( l1length == 3 ) { // "$CMD,ok,V=70*E1\r"
        printf("T1Firm-recvLon $CMD step-3 ok: %ld[%s]\n", list1.size(), data);
    } else if ( l1length == 37 ) { // "$CMD,ok,Factory Set to:I=1,FN,SanFeng01,RA,19,19,50,170,AR,8,2,12,WD,10,11,SW,23,23,23,23,SX,100,100,SL,25,25,SC,20,60,RS,1,2,5,VA,75*E1\r"
        printf("T1Firm-recvLon $CMD step-5 ok: %ld[%s]\n", list1.size(), data);
    } else {
        printf("T1Firm-recvLon $CMD step-x error: %ld[%s]\n", list1.size(), data);
    }
*/
    return -10;
}

/* reset the 415 when the IMU lost
 * 2026.04.11: creat by Leo
 * 2026.04.11: modified by Xs
 * 2026.05.16: moved in T1Firm.cpp
 *
void T1Firm::reset415() {
    printf("T1Firm-reset415 begin.\n");
    sprintf(StrCmdSend, "@CMD,R\r\n");
    printf("T1Firm-reset415 end.\n");
}*/

MapPoint T1Firm::GetPoint(char name) {
    MapPoint tmppoint;
//printf("T1Act-SetStatus() begin. %c, %d\n", name, val);
    switch (name) {
    case 'I':
        return myRoboI;
        break;
    case 'R':
        return myPosRTK;
        break;
    default:
        printf("T1Firm-GetPoint unknown error, name=%c\n", name);
        break;
    }
//printf("T1Act-SetStatus() end. WS5=%d, WS6=%d, WSR=%d, WST=%d\n", WorkStatus5, WorkStatusSimu, WorkStatusR, WorkStatusT);
    return tmppoint;
}

void T1Firm::SetStatus(char name, int val) {
//printf("T1Act-SetStatus() begin. %c, %d\n", name, val);
    switch (name) {
    case 'K':
        FlagRtkReset = val;
        break;
    case 'L':
        meLogType = val;
        break;
    case 'T':
        meTimeout415 = val;
        break;
    default:
        printf("T1Act-SetStatus unknown error, name=%c, val=%d\n", name, val);
        break;
    }
//printf("T1Act-SetStatus() end. WS5=%d, WS6=%d, WSR=%d, WST=%d\n", WorkStatus5, WorkStatusSimu, WorkStatusR, WorkStatusT);
}


/* log RTK message for collect error in them
 * 2026.05.16: moved in T1Firm.cpp
 */ 
void T1Firm::LogLonBug(const char * data) {
    // meLogType: 0=off, 1=LogLonBug only, 2=LogBug only, 3=both
    if (meLogType != 1 && meLogType != 3) {
        return;
    }
    
    // Use unified T1Utils time functions
    std::string currentTime = T1Utils::getCurrentYYYYMMDD() + " " + T1Utils::getCurrentHHMMSS();
    std::string date = T1Utils::getCurrentYYYYMMDD();

    std::string ssid = T1Utils::meSsid;
    std::string filename = "../Log/" + ssid + "Lon" + date + ".log";

    std::ofstream fileout(filename, std::ios::app);
    if (!fileout.is_open()) {
        std::cerr << "T1Firm-LogLonBug, Can not open log file: " << filename << std::endl;
        return;
    }

    fileout << currentTime << " [" << data << "]" << std::endl;
    fileout.close();
}

/* log some thing
 * 2026.05.16: moved in T1Firm.cpp
 */ 
void T1Firm::LogLonDelta(double delta) {
    if (meLogType != 1 && meLogType != 3) {
        return;
    }

    // Use unified T1Utils time functions
    std::string mydate = T1Utils::getCurrentYYYYMMDD();
    std::string hhmmss = T1Utils::getCurrentHHMMSS();
    uint64_t timeUs = T1Utils::getCurrentTimeUs();

    char mytime[30];
    snprintf(mytime, sizeof(mytime), "[%s.%06ld]", hhmmss.c_str(), timeUs % 1000000);

    std::string ssid = T1Utils::meSsid;
    char filename[80];
    int ret = snprintf(filename, sizeof(filename), "../Log/%svoltage%s.log",
                       ssid.c_str(), mydate.c_str());
    if (ret < 0 || ret >= (int)sizeof(filename)) {
        std::cerr << "T1Firm-LogLonDelta, filename too long" << std::endl;
        return;
    }

    std::ofstream fileout(filename, std::ios::app);
    if (!fileout.is_open()) {
        std::cerr << "T1Firm-LogLonDelta, Can not open log file:" << filename << std::endl;
        return;
    }

    fileout << "T1Firm:" << mytime << "[deltaV=" << std::fixed << std::setprecision(4)
            << delta << "]" << std::endl;
    fileout.close();
}


/*
-void T1Act::recvLon( const char * data) {
-    char tmpstr[300];
-    static int count=0;
-
-    double utime=0,lon=0,lat=0,alt=0,roll=0,pitch=0,yaw=0;
-    int dist=0,dis_x=0,dis_y=0,dis_h=0,dis_w=0,time=0;
-    int sateA=0;//, sateB=0;
-
-    static uint64_t timepowerup=0, timenow=0, timeGNGGA=0, timeRoll=0, timeSTA=0;
-    struct timespec ts;
-    clock_gettime(CLOCK_REALTIME, &ts);
-    timenow = (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
-    if ( timepowerup == 0 ) timepowerup=timenow; // the time when The1 start up
-
-    //LogLonDelta(data, "STM415");
-    std::string str = data;
-    //printf("================T1Act-recvLon begin: [%s]================\r\n", data);
-    if ( str.size() < 3 && str[0] != '$' ) {
-        posFixedRP = 0;
-        LogLonBug(data);
-        printf("T1Act-recvLon unknown: [%s]\n", data);
-        return;
-    }
-    std::vector<std::string> list1;
-    list1 = T1Utils::split(str,',');
-    size_t l1length=0;
-    l1length = list1.size();
-    //printf("T1Act-recvLon begin: %lu,[%s]\r\n", l1length, str.c_str()); //list1[0].c_str());
-    if ( list1[0] == "$Info" ) {
-        //FlagInfomation = FlagInfomation | 0x80;
-        //printf("T1Act-recvLon $Info: %lu,[%s]\n", l1length, data);//list1[0].c_str());
-    } else if ( list1[0] == "$STA" ) { //I'll reorganize it later. The function remains anonymous
-        timeSTA = timenow;
-        // $STA,ver=0.3.1,bat=12.45,L=0,R=0,C=0,RL=0,RR=0,S1=0,S2=0,Rain=0
-        // $STA,0.3.1,12.45,0,0,0,0,0,0,0,0
-        if ( l1length >= 13 ) {
-            //printf("T1Act-recvLon $STA length : %lu,[%s]\n", l1length, data);
-            try {
-                std::string ver = list1[1];   // "0.3.1"
-                std::string bat = list1[2];   // "12.45"
-                int cfalg = std::stoi(list1[3]);   // Charging marker position
-                int sL    = std::stoi(list1[4]);   // sen_front_left
-                int sR    = std::stoi(list1[5]);   // sen_front_right
-                int sC    = std::stoi(list1[6]);   // sen_center
-                int sRL   = std::stoi(list1[7]);   // sen_rear_left
-                int sRR   = std::stoi(list1[8]);   // sen_rear_right
-                int sS1   = std::stoi(list1[9]);   // sen_stop1
-                int sS2   = std::stoi(list1[10]);  // sen_stop2
-                int sRain = std::stoi(list1[11]);  // sen_rain
-                int gHome = std::stoi(list1[12]);  // Go_home
-                const double parsedVoltage = std::stod(bat);
-                const double alpha = 0.6;
-                if (batteryVoltage <= 0.0) {
-                    batteryVoltage = parsedVoltage;  // first sample
-                } else {
-                    batteryVoltage = alpha * parsedVoltage + (1.0 - alpha) * batteryVoltage;
-                }
-                if (meFact.HardCharging) ProcessBatteryVoltage(bat);
-                else FlagInCharge = cfalg ; // 0: not charging, 1: charging
-                strncpy(meHardwareVersionF415, ver.c_str(), sizeof(meHardwareVersionF415)-1);
-                if ( sRain == 0 || gHome ==1) {
-                    FlagGoHome = 1;
-                    if ( FlagTestNight!=3 )
-                        sprintf(StrCmdSend, "@CMD,g\r\n");
-                    //printf("T1Act-recvLon $STA: Rain detected! Triggering return home.\n");
-                }
-                if ( meFact.MagnetStop == 0 ) {
-                    bool magnetHit   = ( sL==1 || sR==1 || sC==1 || sRL==1 || sRR==1 );
-                    bool boundaryHit = ( sS1==0 || sS2==0 );
-                    if ( magnetHit || boundaryHit ) {
-                        if ( FlagTestNight!=3 ) {
-                            printf("T1Act-recvLon $STA stop. [%s]\n", data);
-                            sprintf(StrCmdSend, "@CMD,g\r\n");
-                        }
-                    }
-                }
-            } catch (const std::exception &e) {
-                sprintf(tmpstr,"T1Act-recvLon $STA parse failed, [%s]", data);
-                T1Utils::LogBug(tmpstr);
-                return;
-            }
-        } else {
-            sprintf(tmpstr,"T1Act-recvLon $STA length failed, [%s]", data);
-            T1Utils::LogBug(tmpstr);
-        } 
-    } else if ( list1[0] == "$KeySignal" ) {
-        // Handle key signal from F415 board: $KeySignal,home\r\n
-        if ( l1length >= 2 ) {
-            try{
-                std::string keyStr = list1[1];
-                // Remove trailing \r\n
-                auto end = keyStr.find_last_not_of("\r\n");
-                if (end != std::string::npos) {
-                    keyStr.erase(end + 1);
-                }
-                if ( keyStr == "home" || keyStr == "h" ) {
-                    Key415Signal = 'h';  // Return home
-                // } else if ( keyStr == "stop" || keyStr == "0" ) {
-                //     Key415Signal = '0';  // Stop all
-                // } else if ( keyStr == "start" || keyStr == "2" ) {
-                //     Key415Signal = '2';  // Start work
-                // } else if ( keyStr == "pause" || keyStr == "p" ) {
-                //     Key415Signal = 'p';  // Pause
-                } else if ( !keyStr.empty() ) {
-                    Key415Signal = keyStr[0];  // Use first character as key
-                }
-                printf("T1Act-recvLon F415 key pressed: %s -> %c\n", keyStr.c_str(), Key415Signal);               
-            } catch (const std::exception &e) {
-                T1Utils::LogBug("T1Act-parse $roll failed");
-                T1Utils::LogBug(data);
-                
-                return;
-            }
-        }
-    } else if ( list1[0] == "$sensor" ) {
-        // $sensor,L=%d,R=%d,C=%d,RL=%d,RR=%d,S1=%d,S2=%d,Rain=%d
-        if ( meFact.MagnetStop == 0 ) {
-            try {
-                //printf("sensor: %s\r\n", data);
-                int hasOne = 0;
-                int hasZero = 0;
-                for ( size_t i = 1; i < l1length && i <= 5; i++ ) {
-                    std::string item = list1[i];
-                    auto pos = item.find('=');
-                    if ( pos != std::string::npos ) {
-                        std::string valStr = item.substr(pos + 1);
-                        auto end = valStr.find_last_not_of("\r\n");
-                        if (end != std::string::npos) valStr.erase(end + 1);
-                        int val = std::stoi(valStr);
-                        if ( val == 1 ) hasOne = 1;
-                    }
-                }
-                for ( size_t i = 6; i < l1length && i <= 8; i++ ) {
-                    std::string item = list1[i];
-                    auto pos = item.find('=');
-                    if ( pos != std::string::npos ) {
-                        std::string valStr = item.substr(pos + 1);
-                        auto end = valStr.find_last_not_of("\r\n");
-                        if (end != std::string::npos) valStr.erase(end + 1);
-                        int val = std::stoi(valStr);
-                        if ( val == 0 ) hasZero = 1;
-                    }
-                }
-                if ( hasOne || hasZero ) {
-                    if ( FlagTestNight!=2 );
-                    sprintf(StrCmdSend, "@CMD,g\r\n");
-                    printf("T1Act-recvLon Magnet sensor triggered STOP!\n");
-                }
-             } catch (const std::exception &e) {
-                T1Utils::LogBug("T1Act-recvLon sensor parse exception");
-                T1Utils::LogBug(data);
-                return;
-            }
-        }
-    } else if ( list1[0] == "$status" ) {
-        try{
-            if ( l1length >= 2 ) {
-                std::string voltageStr = list1[1];
-                auto end = voltageStr.find_last_not_of("\r\n");
-                if (end != std::string::npos) {
-                    voltageStr.erase(end + 1);
-                } else {
-                    voltageStr.clear();
-                }
-                ProcessBatteryVoltage(voltageStr);
-            }
-            if ( l1length >= 4 && list1[2] == "ver" ) {
-                std::string verStr = list1[3];
-                auto end = verStr.find_last_not_of("\r\n");
-                if (end != std::string::npos) {
-                    verStr.erase(end + 1);
-                }
-                strncpy(meHardwareVersionF415, verStr.c_str(), sizeof(meHardwareVersionF415) - 1);
-                //printf("T1Act-rec updated: %s\n", meHardwareVersionF415);
-            }
-        }catch (const std::exception &e) {
-            T1Utils::LogBug("T1Act-parse $status failed");
-            T1Utils::LogBug(data);
-            return;
-        }
-    } else if ( list1[0] == "$roll" ) {
-        //printf("T1Act-recvLon $roll: %lu,[%s]\n", l1length, list1[0].c_str());
-        timeRoll = timenow;
-        if ( l1length == 13 ) {// "$roll,-0.07,pitch,-0.61,yaw,356.13,count,1234546,x,y,yaw,baselon,baselat\r"
-            try {
-            roll = std::stod(list1[1]) * 100;
-            pitch = std::stod(list1[3]) * 100;
-            yaw = std::stod(list1[5]) * 100;
-            FlagIMUcount = std::stod(list1[7]);
-            myRoboI.roll = roll/100;
-            myRoboI.pitch = pitch/100;
-            myRoboI.yaw =  -yaw/100;
-            if ( myRoboI.yaw < 0 ) myRoboI.yaw += 360;
-            //while ( myRoboI.yaw >= 360 ) myRoboI.yaw -= 360;
-            LogLonBug(data);
-            myRoboM.yaw = myRoboI.yaw + myDiffMI.yaw;
-            if ( myRoboM.yaw >= 360 ) myRoboM.yaw -= 360;
-            if ( myRoboM.yaw < 0 ) myRoboM.yaw += 360;
-
-            /*if ( WorkStatus5!=-1 ) { // SMD machine, Testing real Work
-                myRoboM.fx = std::stof(list1[8]);   // when use T=x,y
-                myRoboM.fy = std::stof(list1[9]);   // when use T=x,y
-                myRoboM.yaw = std::stod(list1[10]);   // when use T=x,y
-                myBase415.lon = std::stod(list1[11]) * PRECISION;   // when use T=x,y// +1;//* 100; // I don't know why must +1
-                myBase415.lat = std::stod(list1[12]) * PRECISION;   // when use T=x,y// +1;//* 100;
-                }*/
/*            } catch (const std::exception &e) {
-                T1Utils::LogBug("T1Act-parse $roll failed");
-                T1Utils::LogBug(data);
-                return;
-            }
-            //line1 = QString::asprintf("T1Act-recvLon $roll-a ok: Robo(%.8f,%.8f) myBase415(%lld,%lld)",list1[11].toDouble(),list1[12].toDouble(),myBase415.lon,myBase415.lat);
-            //std::cout << line1;
-
-            FlagInfomation = FlagInfomation | 0x01;
-            //myMower6.yaw = myMower5.yaw - mapBase.yaw;
-           // if (TestNight==1 && count%5==0 ) printf("T1Act-recvLon ok: %ld,[%s]\n", list1.size(), data);// << "IMU:" << myRoboI.roll << myRoboI.pitch << myRoboI.yaw << "Robo:" << myRoboM.yaw << myDiffMI.yaw;
-        } else {
-            sprintf(tmpstr,"T1Act-recvLon $roll-x error,[%s]", data);
-            T1Utils::LogBug(tmpstr);
-            //printf("T1Act-recvLon $roll: %lu,[%s]\r\n", l1length, list1[0].c_str());
-        }
-        //std::cout << "T1Act-recvLon $roll:"<< list1.size() << line ;
-    } else if ( list1[0][1] == 'G' ) {  // GNGGA, GPGGA
-    //} else if ( list1[0] == "$GNGGA" ) {
-        timeGNGGA = timenow;
-        if (meFact.RtkSetFlag == 0) {
-            ;
-        } else if ( l1length == 15 || l1length==14 ) { // 14 means no base RTK's signal
-        //15,"$GNGGA,20240416063817.11,120.38758450,31.52241876,16.4509,356.13,-0.61,356.13,0.008,-0.07,5,0,18,18,-0.154,-0.164,-0.016,-0.004,0.006,0.000,10,0*4B\r"
-        //14,"$GNGGA,081817.00,3134.70257780,N,12020.97268343,E,1,28,0.6,32.0376,M,7.4475,M,,*75"
-            //printf("T1Act-recvLon GNGGA: %ld,[%s]\n", l1length, data);
-            if ( FlagTestNight==1 && count%5==0 ) {
-                sprintf(tmpstr,"T1Act-recvLon ok: %ld,[%s]\n", list1.size(), data);
-                LogLonBug(tmpstr);
-            }
-            if ( 1 != sscanf(list1[1].c_str(),"%lf",&utime) )
-                printf("T1Act-recvLon sscanf error.\n");
-            try {
-                utime = std::stod(list1[1]);
-                lat = std::stod(list1[2]);
-                if ( list1[3][0]=='S' ) lat *= -1;
-                lon = std::stod(list1[4]);
-                if ( list1[5][0]=='W' ) lon *= -1;
-                myPosRTK.alt = std::stod(list1[9]) * 100;
-                posFixedRP = std::stoi(list1[6]);
-                posFixedRY = std::stod(list1[11]);
-                sateA = std::stod(list1[7]);
-                StataNum = sateA;
-                //sateB = list1[13].toInt();
-                rtkFixedPolaris = posFixedRP;
-            } catch (const std::invalid_argument& e) {
-                LogLonBug(data);
-                std::cerr << "error argumetn" << e.what() << std::endl;
-            } catch (const std::out_of_range& e) {
-                LogLonBug(data);
-                std::cerr << "out of range" << e.what() << std::endl;
-            }
-            if ( posFixedDemo != -1 ) posFixedRP = posFixedDemo;
-            myPosRTK.lon = pMap->TransLonLat(lon) * PRECISION;
-            myPosRTK.lat = pMap->TransLonLat(lat) * PRECISION;
-            // somethig error when lon/lat is 0
-            if ( lon < -180*PRECISION || lon > 180*PRECISION ) {
-                LogLonBug(data);
-                std::cout << "T1Act-recvLon $GNGGA-a: " << data << l1length << lon << lat << 180*PRECISION << std::endl;
-                return;
-            }
-            if ( lat < -90*PRECISION || lat > 90*PRECISION ) { //"$GNGGA,,,,,,0,,,,,,,,*78\r"
-                LogLonBug(data);
-                std::cout << "T1Act-recvLon $GNGGA-b: " << data << l1length << lat << std::endl;
-                return;
-            }
-            if ( posFixedRP == 4 ) {
-                //myRTKF.lon = meScaleLon * ( myRTK.lon - mapBase.lon );
-                //myRTKF.lat = meScaleLat * ( myRTK.lat - mapBase.lat );
-                myPosRTKF.lon = 1 * ( myPosRTK.lon - mapBase.lon );
-                myPosRTKF.lat = 1 * ( myPosRTK.lat - mapBase.lat );
-                myPosRTKF.alt = myPosRTK.alt;
-            }
-            if ( WorkStatusT >= 0 )
-                MapByTrail(data);
-            FlagInfomation = FlagInfomation | 0x02;
-            //if ( count%10 == 0 ) std::cout << "T1Act-recvLon $GNGGA ok: " << list1.size() << line;
-            //if ( FlagTestNight==1 && count%5==0 ) 
-                //printf("T1Act-recvLon ok: %ld,[%s]\n", list1.size(), data);// << "IMU:" << myRoboI.roll << myRoboI.pitch << myRoboI.yaw << "Robo:" << myRoboM.yaw << myDiffMI.yaw;
-        } else {
-            posFixedRP = -1;
-            if (FlagTestNight==1) {
-                sprintf(tmpstr,"T1Act-recvLon $GNGGA-x error:%ld,[%s]\n", l1length, data);
-                T1Utils::LogBug(tmpstr);
-            }
-        }
-    } else if ( list1[0] == "$CMD" ) {
-        //printf("T1Act-recvLon $CMD step1: %ld,[%s]\n", list1.size(), data);
-        if ( l1length == 4 ) {  // "$CMD,ok,T=10,100*E6"
-            if ( list1[2][0] == 'T' ) {// "$CMD,ok,T=10,100*E6"
-                if ( list1[1] == "ok" ) {  // "$CMD,ok,T=10,100*E6"
-                    printf("T1Act-recvLon $CMD step-a: %d, [%s]\n", meWorkstep, data);// << yaw/100;// << line;
-                    meWorkstep = 2;  // target received
-                } else if ( list1[1] == "reach" ) { // "$CMD,reach,T=10,100*E6"
-                    if ( meWorkstep == 2 ) {
-                        printf("T1Act-recvLon $CMD step-b: %d, [%s]\n", meWorkstep, data);// << yaw/100;// << line;
-                        meWorkstep = 3;  // target reached
-                    }
-                } else {  // "$CMD,error,T=10*E6"
-                    printf("T1Act-recvLon $CMD step-c: %d, [%s]\n", meWorkstep, data);// << yaw/100;// << line;
-                }
-            }
-        } else if ( l1length == 3 ) { // "$CMD,ok,V=70*E1\r"
-            printf("T1Act-recvLon $CMD step-3 ok: %ld[%s]\n", list1.size(), data);
-        } else if ( l1length == 37 ) { // "$CMD,ok,Factory Set to:I=1,FN,SanFeng01,RA,19,19,50,170,AR,8,2,12,WD,10,11,SW,23,23,23,23,SX,100,100,SL,25,25,SC,20,60,RS,1,2,5,VA,75*E1\r"
-            printf("T1Act-recvLon $CMD step-5 ok: %ld[%s]\n", list1.size(), data);
-        } else {
-            printf("T1Act-recvLon $CMD step-x error: %ld[%s]\n", list1.size(), data);
-        }
-    } else if ( list1[0] == "$stop" ) {
-        //printf("T1Act-recvLon $stop step1: %ld,[%s]\n", list1.size(), data);
-        if ( l1length == 4 ) {  // "$stop,[RotR],yaw,[20.016621]"
-            if ( list1[1] == "[RotL-Z]" )
-                FlagStop = 1;
-            else if ( list1[1] == "[RotR-C]" )
-                FlagStop = 2;
-            else if ( list1[1] == "[ModiL X+]" )
-                FlagStop = 3;
-            else if ( list1[1] == "[ModiR X-]" )
-                FlagStop = 4;
-            else if ( list1[1] == "[ActObs Q]" )
-                FlagStop = 6;
-            else if ( list1[1] == "[ActObs E]" )
-                FlagStop = 7;
-            else
-                printf("T1Act-recvLon stop unknown:%s\n", data);
-            yaw = std::stod(list1[3].substr(1)) * 100;
-            myRoboI.yaw =  -yaw/100;
-            while ( myRoboI.yaw < 0 ) myRoboI.yaw += 360;
-            while ( myRoboI.yaw >= 360 ) myRoboI.yaw -= 360;
-            myRoboM.yaw = myRoboI.yaw + myDiffMI.yaw;
-            while ( myRoboM.yaw >= 360 ) myRoboM.yaw -= 360;
-            while ( myRoboM.yaw < 0 ) myRoboM.yaw += 360;
-            //printf("T1Act-recvLon stop ok: %ld[--], FlagStop=%d, myRoboM.yaw=%d\n", list1.size(), FlagStop, myRoboM.yaw);            
-         } else {
-            printf("T1Act-recvLon stop error: %ld[%s]\n", list1.size(), data);
-        }
-    } else if ( list1[0] == "$command" ) {
-        posFixedRP = 0;
-        //printf("command\n");
-        //T1Utils::LogBug(data);
-    } else {
-        posFixedRP = 0;
-        T1Utils::LogBug(data);
-        //std::cout << "T1Act-recvLon unknown: " << l1length << ",[" << data << "]" << std::endl;
-        return;
-    }
-    
-    if ( timenow-timepowerup > 5*meTimeout415 ) {
-        //printf("T1Act-recvLon timeout a: %ld,%ld,%ld,%ld\n", timenow, timeRoll, timeGNGGA, timeSTA);
-        if ( FlagReset415 == 1 ) {
-            FlagPause = 0;
-        }
-        if ( timenow-timeGNGGA > meTimeout415 ) {
-            if (meFact.RtkSetFlag != 0) {
-                sprintf(tmpstr,"T1Act-recvLon GNGGA timeout, timenow=%ld, timeGNGGA=%ld, diff=%ld", timenow, timeGNGGA, timenow-timeGNGGA);
-                T1Utils::LogBug(tmpstr);
-                // should tell 415 to reboot RTK
-            }
-        }
-        if ( timenow-timeRoll > meTimeout415 ) {
-            sprintf(tmpstr,"T1Act-recvLon Roll timeout, timenow=%ld, timeRoll=%ld, diff=%ld", timenow, timeRoll, timenow-timeRoll);
-            T1Utils::LogBug(tmpstr);
-            // should tell 415 to reboot IMU
-            timepowerup = timenow;
-            FlagReset415 = 1;
-            FlagPause = 1; 
-            reset415();
-        }
-        /*if ( timenow-timeSTA > meTimeout415 ) {
-            sprintf(tmpstr,"STA timeout, timenow=%ld, timeSTA=%ld, diff=%ld", timenow, timeSTA, timenow-timeSTA);
-            LogBug(tmpstr);
-            printf("T1Act-recvLon timeout: %s\n", tmpstr);
-            // should tell 415 to reboot sensors
-        }*/
        //printf("T1Act-recvLon timeout b: %ld,%ld,%ld,%ld\n", timenow, timeRoll, timeGNGGA, timeSTA);
/*    }
-
-    //if ( count%10 == 0 )
-    //std::cout << "T1Act-recvLon step4c: " << posFixedRY << myMower5.roll << myMower5.pitch << myMower5.yaw << posFixedRP << myMower5.lon << myMower5.lat << myMower5.alt;
-    count ++;
-
-    // do not change the format for history reason!!!
-    //char line1[1000];
-    //sprintf(line1,"%s",data);//"LON:[%9.7f] LAT:[%9.7f] ALT:[%4.2f] roll:[%4.2f] pitch:[%4.2f] yaw:[%d.%d]",
-    //                          myMower5.lon/(double)PRECISION,myMower5.lat/(double)PRECISION,myMower5.alt/100.0,roll/100,pitch/100,posFixedRP,posFixedRY);// yaw/100);
-    if ( utime != 0 )
-        ;//std::cout << "T1Act-recvLon step5: " <<  mapBase.yaw << myMower6.yaw << myMower5.yaw << myMower5.lon << myMower5.lat << myMower5.alt/100 << myMower5.roll/100 << myMower5.pitch/100
-       //      << (ulong)utime << fixed << sateA << sateB;
-    //std::cout << "T1Act-recvLon end:" << myMower5.yaw << (int)yaw/100 << line1;// << line;f
-
-}*/
