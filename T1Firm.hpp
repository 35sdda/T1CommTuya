#ifndef T1FIRM_H
#define T1FIRM_H

#include <vector>
//#include <stdio.h>
//#include <string>
#include "T1Common.hpp"

class T1Firm
{
private:

public:
    T1Firm();
    ~T1Firm();
    int recvLon(const char*);
    void SetStatus(char, int);
    MapPoint GetPoint(char);
    STAStatus GetSTAStatus() const { return mySTAStatus; }

private:
    int processSTA(const std::vector<std::string>, uint64_t);
    int processRoll(const std::vector<std::string>, uint64_t);
    int processIMU(const std::vector<std::string>, uint64_t);
    int processStop(const std::vector<std::string>, uint64_t);
    int processGNGGA(const std::vector<std::string>, uint64_t);
    int processCMD(const std::vector<std::string>, uint64_t);
    //void reset415();
    void LogLonBug(const char *);
    //void LogLonDelta(const char *, const char *);
    void LogLonDelta(double delta);
    //void LogBug(const char *);

private:
    int meLogType=0;
    int meTimeout415=0;
    //int FlagReset415 = 0;
    //int FlagIMUcount = 0; // the count of F415 runtime
    MapPoint myRoboI;   // the data in IMU
    MapPoint myPosRTK;   // the position data in RTK
    int FlagRtkReset=0;  // =1, has rtk module and can be reset when timeout
    STAStatus mySTAStatus;  // STA status data from 415

};


#endif // T1FIRM_H
