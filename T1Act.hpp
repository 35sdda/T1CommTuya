#ifndef T1ACT_HPP
#define T1ACT_HPP

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <errno.h>
#include <sys/time.h>
#include <vector>
#include "T1Map2.hpp"
#include <unistd.h>  
#include <regex>      
#include <sys/stat.h>
#include <chrono>
//#include <iomanip>
#include "T1Map.hpp"
#include "T1Vision.hpp"
//#include "T1Common.hpp"

//#define DEFAULT_PORT    1111    //default port
//#define BUFF_SIZE       1024    //buffer size
//#define SELECT_TIMEOUT  100000       //select() timeout seconds

//#define PRECISION (10000000.0)

#define RATEGO (1)
#define RATEGOX (1)
#define RATEROT (6)
//#define STEPLON (25)
#define WALKPWM (2) // percent of PWM sent to motor
#define WALKSLOW (50) // percent of speed when walk modify
#define WALKBACK (2) // percent of PWM sent to motor
#define WALKBACKSLOW (50) // percent of speed when walk back modify
#define DELAYPRE (5)

#define STEPLINE (30) // must big than 20*MODIFYL in acturel place
#define STEPSAME (20)  // less than this distance(20cm), think is the same point
#define NOTMODIFY (2)    // not rotate if the degree less than notrot,     SuMeiDa
#define NOTROT (5+(NOTMODIFY))      // not rotate if the degree less than notrot
//#define NOTMODIFY (5)    // not rotate if the degree less than notrot,    DeLin
//#define NOTROT (20+(NOTMODIFY))      // not rotate if the degree less than notrot

#define TIMEARCLE (1080)
#define RATERADIUS (50)  // arcle moving's radius: "@CMD,q=5,6,7\r\n"

#define SPEEDWALKX (20.0)  // cm/s,  speed when walk modify
#define SPEEDROTX (2.0)  // degree/s,  speed when rotate modify
#define SPEEDWALK (26.0)  // cm/s
#define SPEEDROT (16.3)  // degree/s
/*
typedef struct PointRobo {
    float fx=0.0;
    float fy=0.0;
    int yaw=0;
    int roll=0;
    int pitch=0;
    char flagPosSetBy = '='; // the mower's position set by WheelOdometry/RTK/AprilTag/VIO, W/R/A/O, default is '='
    uint64_t timeSet = 0;
}PointRobo;
*/
class Factory
{
public:
    char Factory[100] = "TheOne";  //name of factory
    int DriveWheel = 0; // 0: rearwheel drive, 1: frontwheel drive, 2: 4-wheel drive 
    int ArcleL = 0;  // the radius when around the obstacle form Left
    int ArcleR = 0;
    int ArcleRadius = 0; // 100cm
    int ArcleDegree = 0;  // 180 or less
    int AngleRot = 0; // will rotate when large than this angle
    int AngleModi = 0; // will modify when large than this angle
    int DiffModify = 0; // wheel's difference when walk modify
    int WheelDiffl = 0;  // diffrence between L/R wheels
    int WheelDiffr = 0;
    int SpeedWalk1 = 0;  // W: Walking Speed
    int SpeedWalk2 = 0;  // X: Forword Modify Speed
    int SpeedWalk3 = 0;
    int SpeedWalk4 = 0;
    int SpeedXYx = 0; // speed diffrence X/Y
    int SpeedXYy = 0;
    int StepLinex = 50; // the step between walk lines in direction X/Y
    int StepLiney = 50; // can not be 0, or will cause error when myMap::myMap() {GetRoute1()}
    int StepLinec = 15;  // width of the cutter's radius
    int StepLineg = 25;  // size of the grid map's unit 
    int StepLinea = 30;  // begin angle of the RoutePlan2 
    int StepCloseS = 0; // think as same point when less than this
    int StepCloseL = 0;
    int RtkSetFlag = 0; // How to set X,Y,angle by RTK    flag/time/angle
    int RtkSetTime = 0;
    int RtkSetAngle = 0;
    int VisAlert = 0;  // less than this will stop and round it, 0 means not work,           cm
    int MagnetStop = 0;  // Magnetic sensor stop mode: 0=stop when detected, 1=stop when not detected
    int RtkMntDiffX = 0; // the RTK antenna mount postion from the two wheels rotation center point
    int RtkMntDiffY = 1;
    float AprilTagSize1 = 0.1; // the size of the AprilTag, meter
    float AprilTagSize2 = 0.2; // the size of the AprilTag, meter
    int ATagRangeA = 80;  // only detect AprilTag inside this angle,  degree
    int ATagRangeD = 300; // only detect AprilTag inside this distance,  cm
    int StationLength = 100;  // the distance to walk when StationOut
    int AvoidThresh = 50; // Threshold of obstacle avoidance, cm
    int AvoidFreq = 1000; // Frequency of obstacle avoidance
    int HardCharging = 0; // 0:  not HardWare detect charging, 1: HardWare detect charging
    int SpeedAdjustMax = 20; // max speed adjustment value for testing, cm/s
    int SpeedAdjustMin = -20; // min speed adjustment value for testing, cm/s
    float SpeedAdjustSpeed = 1.9; // speed adjustment speed, cm/s/s
    int TestVision = 0; // 0: not test, 1: test vision
};

class T1Act {
protected:
    //FlagInCharge = 0;
    //int IsCharing = 0;

    int meStageNorth=0; // the stage of set North, 0 means not set the North
    int meAngleNorth = 0; // the angle of North
    int meAngleStation = 0; // the angle of charging Station
    int meDiffNorth = 0; // the diff between IMU and North
    int meDiffStation = 0; // the diff between IMU and charging Station

    //int meKarlman=0;  // decide the karlman filter gain
    int meKarlmanR=0; //RTK Karlman gain time threshold
    int meKarlmanA=0; //Visual AprialTag Karlman gain time threshold
    int meKarlmanS=0; //Speed adjust Karlman gain time threshold
    int meTimeout415=3000; // the timeout of 415->X3
    int mevisEnterCnt = 0;
    Factory meFact;

    //std::vector<MapPoint> myMapRemote;  // used when log map
    //std::vector<MapPoint> myMapBorder;  // boarder points from TrailBorder.map
    //std::vector<MapPoint> myRoutePlan;  // route points when scan ma
    
    // the follow 3 point must bu update immediately after GetRoute()
    MapPoint mapMax;   // the maxinum point in map
    MapPoint mapMin;   // the mininum point in map
    MapPoint mapBase;  // the base point in map

    MapPoint myBase415; // base remenbered in F415 board
    MapPoint myPosRTK;   // the position data in RTK
    MapPoint myPosRTKF;  // the lasted fixed position data in RTK
    
    PointRobo myPosVisA;  // the position data by Viaual AprialTag
    PointRobo myPosVisO;  // the position data by Viaual VIO
    Wall mySgbmWall;  // the wall data by SGBM

    int posFixedRP = -1;  // RTK point fixed
    //int posFixedRY = -1;  // RTK yaw fixed
    int posFixedVA = -1;  // Vision Detect
    int posFixedVB = -1;  // The Flag of SGBM
    int posFixedDemo = -1;
    //int VisionRadar = 500;
    int rtkFixedPolaris = -1;  // RTK point fixed for Polaris
    int meSavedPolarisAngle = -1;    // The saved northward Angle, -1 indicates uncalibrated
    bool mePolarisCalibrated = false; // Has the northbound direction been calibrated

    PointRobo myRoboI;  // the data in IMU
    PointRobo myMower6; // simulate move on screen
    PointRobo myRoboM;  // the data in CPU's Memory
    PointRobo myRoboC;  // the center point when visiual obstacle Q/E
    PointRobo myDiffMI;  // the diffrence between CPU's Memory and IMU

    PointRobo myRoboVA; // Pointer to vision data from T1All thread
    //int yawbegin;  // the yaw when visual-obstaclce Q/E begin
    
    int meWorkline = 0;
    int meWorkstep = 0;

    int WorkStatusT = -1; // Trail the map remote point, -1:not trail, 0:begin trail, >0:the points' count
    int WorkStatusSimu = -1;  // for simulation, 0:ready,  >0:Working, <0:stop
    int WorkStatusSimuAll = -1;  // WorkAll for simulation, 0:ready,  >0:Working, <0:stop
    int WorkStatusRobo = -1;  // same as WorkStatusR, but only used in work2(), 0:ready,  >0:Working, <0:stop
    int WorkStatusRoboAll = -1;  // Robot All work with Astar, include charge station in/out, 0:ready,  >0:Working, <0:stop
    //int WorkStatus5 = -1;  // SMD real work, goto target by F415 controll, 0:ready,  >0:Working, <0:stop
    //int WorkStatusAll = -1;  // Robot All work, include charge station in/out, 0:ready,  >0:Working, <0:stop
    int WorkStatusR = -1;  // Robot real work, goto target by X3 controll, 0:ready,  >0:Working, <0:stop
    int WorkStatusH = -1;  // Return Home: -1=not started/finished, 0=init, 1=working
    int WorkStatusGH = -1; // GoHome status machine, 0:ready,  >0:Working, <0:stop
    int WorkStatusGB = -1; // GoBorder status machine, 0:ready,  >0:Working, <0:stop
    int WorkStatusStaOut = -1; //-1: not start/finished, 0: init(check obstacles/battery), 1: execute leave, then -1 when finished
    int WorkStatusStaIn = -1; //-1: not start/finished, 0: init, 1: turn face, 2: recognize, 3: turn around, 4: backward, 5: judge charge, then -1 when finished
    int WorkStatusPolar = 1;
    int WorkStatusDetect = 0;  //0=init, 1=forward, 2=backward, 3=done

    // Shared charging flag across all T1Act instances
    int FlagImuCount = 0; // IMU count from F415, used for monitor 415's runtime
    int FlagSetSpeed = 0;
    int FlagGoHome = 0; // break simulationAll/realworkAll to go home
    int FlagPause = 0; // pause simulation and real work
    int FlagStop = 0;  // 0:not stopped,  >0: stopsign for 'z','c','X' and vision obstacle
    int FlagBangRotating = 0;  // 0: not rotate,  1: in rotating
    int FlagInCharge =0; // 0: not charging, 1: charging
    int IsCharing = 0; // 1: detected as charging, 0: not detected as charging
    uint8_t FlagInfomation = 0x00; // signal of IMU,RTK,VIS
    double batteryVoltage = 0.0;//Battery voltage (smoothed)
    double batteryVoltageRaw = 0.0;//Battery voltage (raw, for charging detection)
    STAStatus mySTAStatus;  // STA status data synced from T1Firm
    //int FlagSpeedCount = 0;  // count the speed of signal(IMU,RTK,VIS)
    //int FlagSpeed = 0;  // count the speed of signal(IMU,RTK,VIS)
    int FlagTestNight = 0;

    //float meScaleLon = 40030173.0 / 360; // the round of the earth/360, when use should *cos(lat)
    //float meScaleLat = 40030173.0 / 360; // the round of the earth/360
    
    int meTypeRoutePlan=1;  // how to RoutePlan

    char meStrSendVis[LENGTHSTRSEND] = "\0";
    char meStrSendCmd[LENGTHSTRSEND] = "\0";
    char meStrSendSocket[LENGTHSTRSEND] = "\0";
    
    int batteryLevel;

public:
    int StataNum = 0;
    std::string verStr;
    int meWorklength = 0;
    int Key415Signal = 0;  // Key signal from F415 board, 0 means no key pressed
    // static int FlagInCharge; // 0: not charging, 1: charging
    // static int IsCharing;
    int SensorMagnetTriggered = 0;  // 1=magnet triggered stop condition

    T1Map *pMap;
    //class T1All *pT1All; // Pointer to T1All instance, will be delete sooner or later
    T1Vision *pmyVision; // Pointer to T1Vision instance, will be delete sooner or later
    //Factory meFact;
    char StrFactory[300] = "\0";
    char StrCmdSend[200] = "\0";
    char StrSocketSend[200] = "\0";
    char meHardwareVersionF415[50] = "0.0.0";
    
protected:
    virtual void PosiSetByWheelO(char, int, int);
    virtual void RoboPosChange(char, int, int);
    virtual void RoboPosInitial();
    virtual int RoboAct(char,int);
    virtual int WorkLine(MapPoint);
    virtual int Work();
    virtual int Work2(); // work use RoutePlan2
    
    // void MapByTrail();
    void MapByTrail(const char *);
    void ProcessBatteryVoltage(const std::string& voltageStr);
    
//    std::vector<std::string> split(const std::string& str, char ch);
    //std::string get_hostapd_ssid(const std::string& hostapd_path = "/etc/hostapd.conf");

public:
    T1Act();
    virtual ~T1Act();
    //void Init();
    void GetFactory(const char *);

    virtual int WorkAll();
    virtual int ReturnHome();
    float GetAprilTagSize(int);
    int GetAvoidThresh() const { return meFact.AvoidThresh; }
    int GetAvoidFreq() const { return meFact.AvoidFreq; }

    int CheckRoboActStatus();
    //void MapFromLog();
    //void reset415();

    void getStrSend(char, char*);
    void clearStrSend(char);
    MapPoint GetPoint(char,int);
    void SetPointRobo(char, PointRobo);
    void SetWall(char, Wall);
    void SetPointRTK(char, MapPoint);
    int GetStatus(char);
    void SetStatus(char, int);
    void UpdateFromSTA(const STAStatus& sta);
    void SetHardwareVersionF415(const char *ver);  // Safe-copy version string into meHardwareVersionF415
    const char *GetHardwareVersionF415() const;
    // void ProcessGNGGA();
    void ProcessGNGGA(const char *);

};

#endif // T1ACT_HPP

