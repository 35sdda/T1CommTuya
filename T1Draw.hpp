#include "T1Mower.hpp"
//#include "T1ActRobo2.hpp"
#include "T1ActRoboAll.hpp"
//#include "T1ActRobo.hpp"
//#include "T1ActSMD.hpp"
#include "T1ActSimuAll.hpp"
//#include "T1Map.hpp"

#include <X11/Xlib.h>
#include <X11/Xutil.h>

//#define WINHEIGHT 1000

struct meRotate {
public:
    float scale_x=0;
    float scale_y=0;
    float scale_z=0;
    float angle_x=0;
    float angle_y=0;
    float angle_z=0;
    float dxsin=0;
    float dycos=0;
    int shift_x=0;
    int shift_y=0;
};

class T1Draw {
private:

    //bool Init(void);
    void redrawCurrentFrame();
    
    int DrawGetRotaX(int,int,int);
    int DrawGetRotaY(int,int,int);
    void DrawGrid(int,int,int,int,int,int);
    void DrawRoutePlan(T1Map *);
    void DrawRoutePlan2(T1Map2 *);
    void DrawRoutePlanLine(T1Map2 *);
    // test() drawing in X11 
    void test2();
    void rotate_point(int, int , int , int , float , int* , int* );
 
private:
    char MapName[200]="------";
    T1Mower myAxis;
    meRotate rotation;
    
    //MapPoint mapMax;
    // four context for X11 window
    Display *myDisplay;
    int myScreen;
    Window myWindow;
    GC myGc;
    int winwidth = 1200, winheight = 600;
    int lawnwidth = 5000, lawnheight = 8000;
    
   // loop argument
    float lineAngle = 0;
    int circleX = 320;
    // position argument
    const int lineCenterX = 125;
    const int lineCenterY = 100;
    const int lineLength = 150;
    const int circleY = 120;
    const int radius = 80;
    const int moveStep = 2;

public:
    char SignalKey = 0;

public:

    T1Draw();
    ~T1Draw();
    bool Init(void);
    //void SetShift(int,int);
    void SetWinSize(int,int,int,int);
    void SetShift(int,int);
    void SetScale(float,float,float);
    void SetRotate(float,float,float);
    void SetMapName(const char *);
    void DrawCommon(const char *, const char * ,const char * = "0.0.0");
    
    void DrawCutLine(PointRobo, PointRobo);
    void DrawMower(T1Mower *);
    void DrawGridMap(T1Map2 *);
    void DrawMap(T1Map *);
    void DrawActStatus(T1Act *);
    void DrawTag(MapPoint);
    void DrawMaxMin(MapPoint, MapPoint);
    void DrawMapBase(T1Map *);//MapPoint, MapPoint);
    void DrawMapBorder(MapPoint);
    void DrawMapTrail(MapPoint);
    
    void myFlush();
    

private:
    //struct timeval timeout;
   
//    int serverSocket=0
//    int InitSocket();
//    void SetSockNonBlock(int);

};
