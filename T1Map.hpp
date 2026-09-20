#ifndef T1MAP_HPP
#define T1MAP_HPP

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <errno.h>
#include <sys/time.h>
#include <vector>
#include <regex>
#include <unistd.h>
#include <stdexcept>
#include <cstddef>

#include "T1Common.hpp"

//#define DEFAULT_PORT    1111    //default port
//#define BUFF_SIZE       1024    //buffer size
//#define SELECT_TIMEOUT  100000       //select() timeout seconds

#define PRECISION (10000000.0)

/*class MapPoint
{
public:
    long long lon=0;
    long long lat=0;
    int alt=0;
    int roll=0;
    int pitch=0;
    int yaw=0;
    int fixed=0;
    int sate=0;
    double batteryVoltage=0.0;//Battery voltage
};
*/
class RouteSetting
{
public:
    int StepLinex = 50; // the step between walk lines in direction X/Y
    int StepLiney = 50; // can not be 0, or will cause error when myMap::myMap() {GetRoute1()}
};

class T1Map {
public:
    std::vector<MapPoint> myMapRemote;  // used when log map
    std::vector<MapPoint> myMapBorder;  // boarder points from TrailBorder.map
    std::vector<MapPoint> myRoutePlan;  // route points when scan map
    bool isDefaultMap = false;  // true: Default map
    class T1All *pT1All; // Pointer to T1All instance

protected:
    int meTypeRoutePlan=5;  // how to RoutePlan

//private:
    MapPoint mapMax;
    MapPoint mapMin;
    MapPoint myBase;

    float meScaleLon = 40030173.0 / 360; // the round of the earth/360, when use should *cos(lat)
    float meScaleLat = 40030173.0 / 360; // the round of the earth/360

    RouteSetting meRouSet;

public:
    T1Map();
    ~T1Map();
    virtual void Init();
    virtual int GetRoute(int); 
    virtual int GetLength(char);
    virtual int SetAprilTagChrgStation(int);

    MapPoint GetPoint(char,int);
    
    void MapFromLog(int);
    double TransLonLat(double);
    void TransXY(int *, int *, long long, long long, long long, long long);
    std::vector<std::string> MapGetList(); //const std::string& dir_path) {
    long long MapGetChecksum(); //const std::vector<std::string>& filenames) {
    int MapSelectWrite(const char *);
    std::string MapSelectRead();
    void MapSelect(const char *);

    void SetRouSet(int,int);

protected:
//private:
    virtual void ReadForbidden(MapPoint);
    virtual void ReadAreaPassOnly(MapPoint);
    virtual void ReadAprilTagPos(MapPoint);
    int RouteALat();
    int RouteALon();
    int RouteZLat();
    int RouteZLon();

    virtual int GetRemote();
    int GetBorder();
    void GetScaleLonLat();

    void LogBug(const char *);
    //std::vector<std::string> split(const std::string& str, char ch);

};

#endif // T1MAP_HPP

