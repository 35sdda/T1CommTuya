#ifndef T1All_H
#define T1All_H

//#include <vector>
//#include "T1DecodeNMEA.hpp"
#include "T1Vision.hpp"
#include "T1Firm.hpp"
#include "T1Draw.hpp"
#include "T1All.hpp"
//#include "T1Act.hpp"
#include "T1Utils.hpp"  // commom functions
#include "T1Common.hpp"

#include <thread>
#include <atomic>
#include <mutex>

class T1All
{
private:
    T1Act *pmyAct;
    T1ActSimu myActSimu;
    T1ActSimuAll myActSimuAll;
    T1ActRobo myActRobo;
    T1ActRoboAll myActRoboAll;
    T1Vision myVision;
    T1Firm myFirm;
    T1Map myMap;
    T1Map2 myMap2;
    T1Draw myDraw;
    T1Mower myMower;
    char meTitleName[100];
    char meSerialPort[100]="port /dev/ttyS2";
    int meCountTimer = 0;
    int meMapCurrent = 0; // map select in list
    
    long long meMapChecksum = 0;
    char meHardwareVersionX3[100]="0.0.0";
    char meHardwareVersionF415[100]="0.0.0";
    
    // Map receive state
    bool meMapReceiving = false;
    bool meMapFilenameReceived = false;
    std::ofstream meMapOutFile;
    std::string meMapFilename;
    int meMapLineCount = 0;

public:
    T1All();
    ~T1All();
    PointRobo ProcessVision(char);
    int ProcessFirmSerial(int, const unsigned char *);
    void Draw(void);
    void Work(void);
    string MapGetList();
    std::vector<std::string> GetLogList();
    void MapDeleteDemo();
    void MapSelectDemo(int);
    void MapSwitchNew(const char*);
    void MapSwitch(const char*);
    void MapDelete(const char *);
    void MapRecvLine(const char* line, const std::string& save_path);
    int GetSignal(char);
    //int GetVisualSignal(char);
    void SetStatus(char, int);
    const char *GetString(char);
    std::vector<std::string> GetRoboAllMapBorderData();
    int SetSysTime(const char*, int);
    std::string getHeartBeatStr();;//T1All *pmyAll)
    void CheckVersion(const char* target, const char* const args[], bool background);
    bool CheckZipIntegrity(const std::string& zip_path);


private:
    void GetParamVersion();
    void GetParamHeart();
    void CheckEnviroment(const char*);
    void LowBatRecharge();
    int JudgeWorkStatus();
    std::string get_version();//const std::string& version_path = "../Ver/Version");
    std::string getMaxZip();    
    
    void GetSetting(const char *);
    void GetSettingFactory();
    void workDemoMove();
    void ResetGridParams();
    void LogTraversalStatus();
    
private:
    // vision thread
    void ThreadVision();

    std::thread visThread;
    std::atomic<bool> running;

    //void LogBug(const char *);
    //std::vector<std::string> split(const std::string& str, char ch);

};


#endif // T1All_H
