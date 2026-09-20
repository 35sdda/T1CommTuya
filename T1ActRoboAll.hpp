#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <errno.h>
#include <sys/time.h>
#include <vector>
#include "T1ActRobo.hpp"
#include "T1Vision.hpp"
#include <filesystem>

class T1ActRoboAll : public T1ActRobo {
private:
    int GoHomeSavePoint();
    int GoHomeReadPoint();
    int GoHomeClearPoint();
    void SetCutLine(MapPoint, MapPoint);
    void SetCutGrid();
    void GoHomeGridMapSave();
    void GoHomeGridMapRead();
    void SetCutGridXs();
    int Work2() override;
    //void PolarisSet(int);
    //void RoboPosChange(char,int,int);
    //int RoboAct(char,int) override;
    //int WorkLine(MapPoint) override;
    int JudgeCharge();
    int JudgeFace();
    void SetStationNorth();  // Set charging station direction as North (0 degrees)
    int StationOut();
    // int StationOutTest(); //2026.05.22: annotation by Xs
    int StationIn();
    int GoHome();
    int GoBorder();
    int RecoFace();
    int JudgeFaceNew();
    int JudgeFaceBY();
    //int ProcessVisionResult(int res);
    void SetObst(Wall);
    int StationInVision();
    int StationInVisionXS();
    int StationInRTK();
    void CheckSensors();          // Hub that calls all individual sensor checks
    void CheckSensorsStop();      // Boundary stop sensors (senStop1, senStop2)
    void CheckSensorsCollision(); // 5 collision sensors (L, R, C, RL, RR)
    void CheckSensorsRain();      // Rain sensor (senRain, active-low): go home when detected
    void CheckSensorsCharge();    // Charge sensor (senCharge, active-low): go home when detected
public:
    T1ActRoboAll();
    ~T1ActRoboAll();

    //int WorkLine(MapPoint);
    int WorkTrail();
    int WorkAll() override;
public:
    PointRobo Cleft, Cright;

private:
    int startimuyaw;
    int meWorkline2 = 0;
    int meWorklength2 = 0;
    int flagReverse = 0;  // direction of cut line, 0 means normal direction, 1 means reverse direction
    PointRobo GoHomePoint;
    //int counttime;
    //int countmax;
    float meVisionDistX;
    float meVisionDistY;
    int meVisionYawAngle;
    float targetYaw;

};

