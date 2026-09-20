#ifndef T1ACTROBO_HPP
#define T1ACTROBO_HPP

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <errno.h>
#include <sys/time.h>
#include <vector>
#include "T1Act.hpp"

class T1ActRobo : public T1Act {
protected:
    int AngleStationNorthClear();
    int AngleStationNorthSave();
    int AngleStationNorthRead();
    void NorthSet(int);
    void LogNorth(const char *);
    //void PolarisSet(int);
    int getGainKarlman();
    void PosiSetByRTK();
    void PosiSetByVisA();
    void PosiSetByWheelO(char,int,int) override;
    void RoboPosChange(char,int,int) override;
    void RoboPosInitial() override;
    int RoboAct(char,int) override;
    int WorkLine(MapPoint) override;
    int Work() override;
    bool isAprilTagInView(long long, long long);
    bool Tag2Robot();
    PointRobo Tag2Camera(float, float, int, long long, long long);
    int AprilTagDetctOld();
    void setSpeedRotate(const char, const int, const int);
    void setSpeedWalk(const PointRobo);
    bool IsRotationStable(float, uint64_t);
public:
    T1ActRobo();
    ~T1ActRobo();
    // double TimeForDistanceCm(double dist_cm);
    // int TimeForDistanceMs(int dist_cm);
    // double DistanceFromTime(double time_s);
    // double DistanceFromTimeMs(int time_ms);
    // void LoadDistanceTimeUpt(const char *path);
private:
    // std::vector<std::pair<double,double>> distTimeTable; // (distance_cm, time_s)
    //int counttime;
    //int countmax;

protected:
    char meCmdC = 'C';
    char meCmdZ = 'Z';
    uint64_t meRotationStableStart=0;
};

#endif // T1ACTROBO_HPP


