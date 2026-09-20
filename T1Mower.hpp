#ifndef T1Mower_H
#define T1Mower_H
//#define alarm_blue 399;
//#define alarm_yellow 20;
//#define alarm_red 10;

#include <vector>
#include "T1DecodeNMEA.hpp"
//#include "T1Map.hpp"
//#include "T1Draw.hpp"

class MyRotate {
public:
    float scale_x=0;
    float scale_y=0;
    float scale_z=0;
    float angle_x=0;
    float angle_y=0;
    float angle_z=0;
    float dxsin=0;
    float dycos=0;
};

class MyPoint
{
public:
    int x=0;
    int y=0;
    int z=0;
};

class MyPointBang
{
public:
    int x=0;
    int y=0;
    int z=0;
    int r=0;
    //int type=0;
};

class MyLine
{
public:
    MyPoint begin;
    MyPoint end;
    int type=0;
    int radius=0;
    int steps=0;
    float stepx=0;
    float stepy=0;
    float stepz=0;
};

class MyAlarm
{
public:
    int x1=0;
    int y1=0;
    int z1=0;
    int r1=0;
    int x2=0;
    int y2=0;
    int z2=0;
    int r2=0;
    int distance=0;
protected:
    //int alarm_blue=300;
    //int alarm_yellow=200;
    //int alarm_red=100;

};

class T1Mower
{
public:
    T1Mower();
    ~T1Mower();
    void init(const char *);
    void MoveSet(int,int,int);
    void MoveRotate(float,float);
    void GetSet(MyPoint *, rtkPoint, int );
    void GetRotate(MyPoint*, rtkPoint *, rtkPoint, rtkPoint, int);
    MyPoint RotateAxis(int, MyPoint);
    //int RotateAxisY(int, MyPoint);
    //int RotateAxisZ(int, MyPoint);
    //int RotateAxisVy(int, MyPoint);
    //int RotateAxisVz(int, MyPoint);

    //void BangDetect(int, T1Mower *, QVector<MyAlarm> *);
    //void BangDetectNew(int, T1Mower *, QVector<MyAlarm> *);
    void SetPBangs(void);

private:
    //std::vector<std::string> split(const std::string& str, char ch);
    void GetDefault();

public:
    char filename[100];
    int IDdevice = 0;
    int devtype;
    int noShift = 0;   // without screen shift when draw;
    int meDiff = 0;
    // parameters for two RTKs
    MyPoint fixRTK1;
    MyPoint fixRTK2;
    int IDRtk1 = 0;
    int IDRtk2 = 0;
    // parameters for two Lidars
    MyPoint fixLidar1,fixLidar2;
    std::vector<MyPoint> meLidar1,meLidar2;
    int reverse[2]={1,1};
    int angleb[2]={0,0};
    int anglee[2]={360,360};

    std::vector<MyLine> body;
    MyPoint move;
    //QVector<MyPointBang> pBangs;
    float rotationH=0;
    float rotationV=0;

    //int remotespeed=0;
    //int remoteangle=0;
    int mowerFixedP = 0;
    int mowerFixedA = 0;

private:
    MyPoint axisH;
    MyPoint axisV;
    float rHsin=0;
    float rHcos=0;
    float rVsin=0;
    float rVcos=0;

};


#endif // T1Mower_H
