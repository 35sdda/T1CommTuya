#include "T1Map.hpp"
#include "T1Utils.hpp"
#include <vector>
#include <string>
#include <cstring>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <math.h>
#include <dirent.h>

T1Map::T1Map() {

}

T1Map::~T1Map() {
    
}

void T1Map::Init() {
    printf("\nT1Map-Init, begin. do nothing\n");
    GetRoute(meTypeRoutePlan);
}

MapPoint T1Map::GetPoint(char name, int val) {
    //printf("T1Map-GetPoint begin, name=%c, val=%d\n",name,val);
    switch (name) {
    case 'b':
        return myBase;
        break;
    case 'a':
        return mapMax;
        break;
    case 'i':
        return mapMin;
        break;
    case 'B':
        return myMapBorder[val];
        break;
    case 'R':
        return myMapRemote[val];
        break;
    case 'P':
        return myRoutePlan[val];
        break;
    default:
        printf("T1Map-GetPoint error type[%c,%d]\n", name, val);
        return myBase;
        break;
    }
}

// get the map border's length
int T1Map::GetLength(char name) {
    switch (name) {
    case 'B':
        return myMapBorder.size();
        break;
    case 'R':
        return myMapRemote.size();
        break;
    case 'P':
        return myRoutePlan.size();
        break;
    default:
        //printf("T1Map-GetLength error type[%c], return -1.\n", name);
        return -1;
        break;
    }
}

/** transfer MapTrail.rtk to MapTrail.map
 * 2026.01.20: created by Leo, moved form T1Act to T1Map
 */
void T1Map::MapFromLog(int flagTestNight) {
    std::cout << "\nT1Map-MapFromLog_, begin." << std::endl;
    char tmpstr[200];
    sprintf(tmpstr,"../Map/TrailRemote.rtk");
    std::ifstream filein(tmpstr);
    if (!filein.is_open()) {
        T1Utils::LogBug("T1Map-MapFromLog_, Can not open log TrailRemote.rtk");
        //myMap.GetRoute(meTypeRoutePlan);
        return;
    }
    
    // Use unified T1Utils time functions
    char mapfile[190];
    std::string dateStr = T1Utils::getCurrentYYYYMMDD();
    std::string timeStr = T1Utils::getCurrentHHMMSS();
    sprintf(mapfile,"../Map/Trail%s-%s.map", dateStr.c_str(), timeStr.c_str());
    std::ofstream fileout(mapfile);
    if (!fileout.is_open()) {
        filein.close();
        T1Utils::LogBug("T1Map-MapFromLog_, Can not open TrailBorder20YYMMDDHHMMSS.map");
        GetRoute(0);
        return;
    }    

    int alt, fixed, satellite;
    long long lon,lat,baselon,baselat;

    std::string line;
    std::vector<std::string> list1;
    int count = 0;
    while ( std::getline(filein,line) ) {
        //std::cout << "MyAct-MapFromLog_ step1, reading: [" << line << "]" << std::endl;
        list1 = T1Utils::split(line,',');
        if ( list1.size() < 14 ) continue;  // or <15
        //printf("T1Map-MapFromLog_ step1: %d, %ld[%s]\n", count, list1.size(), line.c_str());
        //"$GNGGA,055556.00,3134.70397170,N,12020.97266526,E,5,38,0.5,26.2219,M,7.4475,M,1.0,0*6D\r"
        lon = T1Utils::TransLonLat(std::stod(list1[4])) * PRECISION;
        if ( list1[5][0]=='W' ) lon*=-1;
        lat = T1Utils::TransLonLat(std::stod(list1[2])) * PRECISION;
        if ( list1[3][0]=='A' ) lat*=-1;
        alt = std::stoi(list1[9]) * 100;
        fixed = std::stoi(list1[6]);
        satellite = std::stod(list1[7]);
        //printf("T1Map-MapFromLog_ step2: %d, %ld, (%lld,%lld,%d)\n", count, list1.size(), lon, lat, alt);
        // somethig error when lon/lat is 0
        if ( lon < -1800000000 || lon > 1800000000 ) {
            std::cout << "T1Map-MapFromLog_ step2a-err: " << count << list1.size() << lon << lat << alt << std::endl;
            continue;
        }
        if ( lat < -900000000 || lat > 900000000 ) {
            std::cout << "T1Map-MapFromLog_ step2b-err: " << count << list1.size() << lon << lat << alt << std::endl;
            continue;
        }
        if ( alt < -1000 ) {  //"320000000,320000000,-500,-300,-100,32600,189"
            std::cout << "T1Map-MapFromLog_ step2c-err: " << count << list1.size() << lon << lat << alt;
            continue;
        }

        //lon = TransLonLat(lon);
        //lat = TransLonLat(lat);
        if ( count == 0 ) {
            baselon = lon;
            baselat = lat;
            sprintf(tmpstr, "%lld,%lld,%d,%d,%d,%d",baselon,baselat,alt,fixed,satellite,count);
            //sprintf(tmpstr, "%d,%d,%d,%d,%d,%d",(int)(baselon*PRECISION),(int)(baselat*PRECISION),(int)alt*100,fixed,satellite,count);
            //sprintf(tmpstr, "%lf,%lf,%d,%d,%d,%d",baselon,baselat,(int)alt*100,fixed,satellite,count);
            printf("T1Map-MapFromLog_ step3o, %d,%ld,[%s]\n", count, list1.size(), tmpstr);
            //printf("T1Map-MapFromLog_ step30, lon:%.8f,%.8f, lat:%.8f,%.8f\n", lon, baselon, lat, baselat);
            fileout << tmpstr << std::endl;
            //fileout2 << tmpstr << std::endl;
        }
        int x,y;
        TransXY(&x,&y,lon,lat,baselon,baselat);
        //int x = cos( abs(lat)*3.14159265/180) * meScaleLon * (lon - baselon) * PRECISION;
        //int y = meScaleLat * (lat - baselat) * PRECISION;

        //printf("T1Map-MapFromLog_ step3: %d, %ld, point(%lf,%lf,%lf), base(%lf,%lf)\n", count, list1.size(), lon, lat, alt, baselon, baselat);
        if ( abs(x) > PRECISION ) {
            printf("T1Map-MapFromLog_ step3a-err: %d,%ld, x=%d\n", count, list1.size(), x);
            continue;
        }
        if ( abs(y) > PRECISION ) {
            printf("T1Map-MapFromLog_ step3b-err: %d,%ld, x=%d\n", count, list1.size(), y);
            continue;
        }

        count++;
        sprintf(tmpstr,"%d,%d,%d,%d,%d,%d",x,y,alt,fixed,satellite,count);
        //printf("T1Map-MapFromLog_ step3:  %d,%ld,[%s]\n", count, list1.size(), tmpstr);
        fileout << tmpstr << std::endl;
        //fileout2 << tmpstr << std::endl;
    }

    printf("T1Map-MapFromLog_ end, Length=%d, base(%lld,%lld)\n", count, baselon, baselat);
    filein.close();
    fileout.close();
    if ( flagTestNight==8 )
        strcpy(tmpstr, "../Map/TrailBoarder.map2");
    else
        strcpy(tmpstr,"../Map/MapDefault.maap");
    if (!T1Utils::copyFile(mapfile, tmpstr)) {
    //if (!T1Utils::copyFile(mapfile, "../Map/MapDefault.maap")) {
        T1Utils::LogBug("T1Map-MapFromLog_, failed to copy TrailBorder.map to MapDefault.maap");
    }
    
    MapSelectWrite(mapfile+7);
    //MapGetChecksum(); // Act in T1All::SetStatus() case'T':
    GetRoute(0);
}

/** Trans Lon/Lat to x,y in cm
 * 2026.01.20: created by Leo, moved form T1Act to T1Map
 */
void T1Map::TransXY(int *px, int *py, const long long lon, const long long lat, const long long baselon, const long long baselat) {
    //*px = (lon - baselon) * PRECISION;
    //*py = (lat - baselat) * PRECISION;
//    *px = ( lon - baselon ) / PRECISION / 360 * 4007401700 * cos(abs(lat/PRECISION)*3.14159265/180);
//    *py = ( lat - baselat ) / PRECISION / 360 * 4000800000;
    *px = ( lon - baselon ) * 400.74017 / 360 * cos(abs(lat/PRECISION)*3.14159265/180);
    *py = ( lat - baselat ) * 400.08000 / 360;
//printf("T1Act-GetScale end. RoboM:(%lf, %lf), Base:(%lf, %lf), XY:(%d,%d)\n", lon, lat, baselon, baselat, *px, *py);
}


void T1Map::SetRouSet(int x, int y) {
    meRouSet.StepLinex = x;
    meRouSet.StepLiney = y;
}

int T1Map::GetRoute(int type) {
    int ret = GetBorder();
    if ( ret <= 0 ) return ret;

    GetScaleLonLat();
    if ( type==0 )
        type=meTypeRoutePlan;
//printf("T1Map-GetRoute begin, type=%d, ret=%d\n", type, ret);
    // scan latitude
    MapPoint tmp; // all is 0,0,0,0,0,0
    myRoutePlan.clear();
    myRoutePlan.emplace_back(tmp); // myBase should be 0,0,0,0,0,0 in the relative coordinates.
//printf("T1Map-GetRoute step0: size:%ld, Base:(%lld, %lld)\n", myRoutePlan.size(), myBase.lon, myBase.lat);// meWorkline << mapMin.lon << mapMax.lon;
    switch (type) {
    case 1:
        RouteZLon();
        break;
    case 2:
        RouteZLat();
        break;
    case 3:
        RouteZLon();
        RouteZLat();
        break;
    case 5:
        RouteALon();
        break;
    case 6:
        RouteALat();
        break;
    case 7:
        RouteALon();
        RouteALat();
        break;
    default:
        std::cout << "T1Map-GetRoute step4, error type:" << type;
        break;
    }

    myRoutePlan.emplace_back(tmp); // add the myBase=0 again
//printf("T1Map-GetRoute step5, size=%ld, Base:(%lld, %lld)\n", myRoutePlan.size(), myBase.lon, myBase.lat);// meWorkline << mapMin.lon << mapMax.lon;

    for ( int i=0; i<myRoutePlan.size(); i++ ) {
        ;//printf("T1Map-GetRoute step6, all RoutePlan: %ld-%d, (%lld, %lld, %d)\n", myRoutePlan.size(), i, myRoutePlan[i].lon, myRoutePlan[i].lat, myRoutePlan[i].alt);
    }

    // scan longitude

    printf("T1Map-GetRoute end, size=%ld, Min(%lld,%lld), Max(%lld,%lld)\n", myRoutePlan.size(), mapMin.lon, mapMin.lat, mapMax.lon, mapMax.lat);
    //std::cout << "T1Map-GetRoute end. myRoutePlan.size()=" << myRoutePlan.size() << mapMin.lat << mapMax.lat;
    //std::cout << "\n";
    
    return myRoutePlan.size();
}

/* Get-Set the scale of cm to LON/LAT 7 bit after point
 */
void T1Map::GetScaleLonLat() {
    //myBase.lon = 1203875485;   wuxi WXSC
    //myBase.lat = 315223142;
    meScaleLon = 1.0;
    meScaleLat = 1.0;
    if ( (int)(myBase.lon/10000) == 120387 )
        ;//meScaleLon = 0.95;
    if ( (int)(myBase.lat/10000) == 31522 )
        ;//meScaleLat = 1.11;
//printf("T1Map-GetScaleLonLat, end. Base:(%lld, %lld), Scale:(%f, %f)\n", myBase.lon, myBase.lat, meScaleLon, meScaleLat);
}

int T1Map::RouteALat() { // A,lat
    MapPoint tmp_point1,tmp_point2,tmp_append;
    int flag = 0;

//printf("T1Map-RouteALat step1, scan longitude to build the RoutePlan, meWorkline=%d, Min:(%lld, %lld), Max:(%lld, %lld)\n", meWorkline, mapMin.lon, mapMin.lat, mapMax.lon, mapMax.lat);
    tmp_point1 = mapMin;
    tmp_point2 = mapMin;
    //return 0;
    for ( int scanlat=mapMin.lat; scanlat<mapMax.lat; scanlat+=meRouSet.StepLiney ) {
        //std::cout << "T1Map-RouteALat step2a, scan map longitude. " << mapMin.lon << scanlon-mapMin.lon << tmp_point1.lat << tmp_point2.lat;
        tmp_point1.lat = scanlat;
        tmp_point2.lat = scanlat;
        flag = 0;
        for ( int i=0; i<myMapBorder.size(); i++ ) {
            if ( myMapBorder[i].lat == scanlat ) {  // found at least one point
                if ( flag == 0 ) { // first point in this longitude
                    tmp_point1.lat = myMapBorder[i].lat;
                    tmp_point2.lat = myMapBorder[i].lat;
                    //std::cout << "T1Map-RouteALat step2b, scan longitude's meWorkline. " << mapMin.lon << scanlon-mapMin.lon << tmp_point1.lat << tmp_point2.lat << myMapRemote[i].lat << i << flag;
                    flag = 1;
                }
                if ( myMapBorder[i].lon < tmp_point1.lon ) tmp_point1.lon = myMapBorder[i].lon;
                if ( myMapBorder[i].lon > tmp_point2.lon ) tmp_point2.lon = myMapBorder[i].lon;
                tmp_point1.alt = i;
                //std::cout << "T1Map-RouteALat step2c, scan longitude's meWorkline. " << mapMin.lon << scanlon-mapMin.lon << tmp_point1.lat << tmp_point2.lat << myMapBorder[i].lat << i << flag;
            }
        }
        if ( ( (((scanlat-mapMin.lat)/meRouSet.StepLiney)) % 2 ) == 0 ) {
            tmp_append.lon = tmp_point1.lon;//meScaleLon * ( tmp_point1.lon - myBase.lon );
            tmp_append.lat = tmp_point1.lat;//meScaleLat * ( tmp_point1.lat - myBase.lat );
            tmp_append.alt = tmp_point1.alt;
            myRoutePlan.emplace_back(tmp_append);
            //myRoutePlan.emplace_back(tmp_point2);
//printf("T1Map-RouteALat step2d1, size=%ld, tmp_append:(%lld, %lld)\n", myRoutePlan.size(), tmp_append.lon, tmp_append.lat);// meWorkline << mapMin.lat << mapMax.lat;
        } else {
            tmp_append.lon = tmp_point2.lon;//meScaleLon * ( tmp_point2.lon - myBase.lon );
            tmp_append.lat = tmp_point2.lat;//meScaleLat * ( tmp_point2.lat - myBase.lat );
            tmp_append.alt = tmp_point2.alt;
            myRoutePlan.emplace_back(tmp_append);
            //myRoutePlan.emplace_back(tmp_point1);
//printf("T1Map-RouteALat step2d2, size=%ld, tmp_append:(%lld, %lld)\n", myRoutePlan.size(), tmp_append.lon, tmp_append.lat);// meWorkline << mapMin.lat << mapMax.lat;
        }
    }

    /*if ( (myRoutePlan.size()/2)%2 == 1 ) {    // longitude scan must be even lines
        myRoutePlan.emplace_back(tmp_point2);
        myRoutePlan.emplace_back(tmp_point1);
        std::cout << "T1Map-RouteALat step3, " << myRoutePlan.size() << tmp_point1.lon << tmp_point1.lat << tmp_point2.lat;// meWorkline << mapMin.lat << mapMax.lat;
    }*/

//printf("T1Map-RouteALat end. myRoutePlan.size()=%ld\n", myRoutePlan.size());// meWorkline << mapMin.lat << mapMax.lat;
    return myRoutePlan.size();

}

int T1Map::RouteALon() { // A,lon
    MapPoint tmp_point1,tmp_point2,tmp_append;
    int flag = 0;

//printf("T1Map-RouteALon step1, scan longitude to build the RoutePlan, meWorkline=%d, Min:(%lld, %lld), Max:(%lld, %lld)\n", meWorkline, mapMin.lon, mapMin.lat, mapMax.lon, mapMax.lat);
    tmp_point1 = mapMin;
    tmp_point2 = mapMin;
    //return 0;
    for ( int scanlon=mapMin.lon; scanlon<mapMax.lon; scanlon+=meRouSet.StepLinex ) {
        //std::cout << "T1Map-RouteALon step2a, scan map longitude. " << mapMin.lon << scanlon-mapMin.lon << tmp_point1.lat << tmp_point2.lat;
        tmp_point1.lon = scanlon;
        tmp_point2.lon = scanlon;
        flag = 0;
        for ( int i=0; i<myMapBorder.size(); i++ ) {
            if ( myMapBorder[i].lon == scanlon ) {  // found at least one point
                if ( flag == 0 ) { // first point in this longitude
                    tmp_point1.lat = myMapBorder[i].lat;
                    tmp_point2.lat = myMapBorder[i].lat;
                    //std::cout << "T1Map-RouteALon step2b, scan longitude's meWorkline. " << mapMin.lon << scanlon-mapMin.lon << tmp_point1.lat << tmp_point2.lat << myMapRemote[i].lat << i << flag;
                    flag = 1;
                }
                if ( myMapBorder[i].lat < tmp_point1.lat ) tmp_point1.lat = myMapBorder[i].lat;
                if ( myMapBorder[i].lat > tmp_point2.lat ) tmp_point2.lat = myMapBorder[i].lat;
                tmp_point1.alt = i;
                //std::cout << "T1Map-RouteALon step2c, scan longitude's meWorkline. " << mapMin.lon << scanlon-mapMin.lon << tmp_point1.lat << tmp_point2.lat << myMapBorder[i].lat << i << flag;
            }
        }
        if ( ( (((scanlon-mapMin.lon)/meRouSet.StepLinex)) % 2 ) == 0 ) {
            tmp_append.lon = tmp_point1.lon;//meScaleLon * ( tmp_point1.lon - myBase.lon );
            tmp_append.lat = tmp_point1.lat;//meScaleLat * ( tmp_point1.lat - myBase.lat );
            tmp_append.alt = tmp_point1.alt;
            myRoutePlan.emplace_back(tmp_append);
            //myRoutePlan.emplace_back(tmp_point2);
//printf("T1Map-RouteALon step2d1, size=%ld, tmp_append:(%lld, %lld)\n", myRoutePlan.size(), tmp_append.lon, tmp_append.lat);// meWorkline << mapMin.lat << mapMax.lat;
        } else {
            tmp_append.lon = tmp_point2.lon;//meScaleLon * ( tmp_point2.lon - myBase.lon );
            tmp_append.lat = tmp_point2.lat;//meScaleLat * ( tmp_point2.lat - myBase.lat );
            tmp_append.alt = tmp_point2.alt;
            myRoutePlan.emplace_back(tmp_append);
            //myRoutePlan.emplace_back(tmp_point1);
//printf("T1Map-RouteALon step2d2, size=%ld, tmp_append:(%lld, %lld)\n", myRoutePlan.size(), tmp_append.lon, tmp_append.lat);// meWorkline << mapMin.lat << mapMax.lat;
        }
    }

    /*if ( (myRoutePlan.size()/2)%2 == 1 ) {    // longitude scan must be even lines
        myRoutePlan.emplace_back(tmp_point2);
        myRoutePlan.emplace_back(tmp_point1);
        std::cout << "T1Map-RouteALon step3, " << myRoutePlan.size() << tmp_point1.lon << tmp_point1.lat << tmp_point2.lat;// meWorkline << mapMin.lat << mapMax.lat;
    }*/

//printf("T1Map-RouteALon end. myRoutePlan.size()=%ld\n", myRoutePlan.size());// meWorkline << mapMin.lat << mapMax.lat;
    return myRoutePlan.size();
}

int T1Map::RouteZLat() { // Z,lat
    MapPoint tmp_point1,tmp_point2,tmp_append;
    int flag = 0;

//printf("T1Map-RouteZLat step1, scan longitude to build the RoutePlan, meWorkline=%d, Min:(%lld, %lld), Max:(%lld, %lld)\n", meWorkline, mapMin.lon, mapMin.lat, mapMax.lon, mapMax.lat);

    tmp_point1 = mapMin;
    tmp_point2 = mapMin;
    flag = 0;
    for ( int scanlat=mapMin.lat; scanlat<mapMax.lat; scanlat+=meRouSet.StepLiney ) {
        //std::cout << "T1Map-RouteZLat step4a, scan map latitude. " << mapMin.lat << scanlat-mapMin.lat << tmp_point1.lon << tmp_point2.lon;
        tmp_point1.lat = scanlat;
        tmp_point2.lat = scanlat;
        flag = 0;
        for ( int i=0; i<myMapBorder.size(); i++ ) {
            if ( myMapBorder[i].lat == scanlat ) {  // found at least one point
                if ( flag == 0 ) { // first point in this latitude
                    tmp_point1.lon = myMapBorder[i].lon;
                    tmp_point2.lon = myMapBorder[i].lon;
                    //std::cout << "T1Map-RouteZLat step4b, scan lanitude's meWorkline. " << mapMin.lat << scanlat-mapMin.lat << tmp_point1.lon << tmp_point2.lon << myMapRemote[i].lon << i << flag;
                    flag = 1;
                }
                if ( myMapBorder[i].lon < tmp_point1.lon ) tmp_point1.lon = myMapBorder[i].lon;
                if ( myMapBorder[i].lon > tmp_point2.lon ) tmp_point2.lon = myMapBorder[i].lon;
                tmp_point1.alt = i;
                //std::cout << "T1Map-RouteZLat step4c, scan latitude's meWorkline. " << mapMin.lat << scanlat-mapMin.lat << tmp_point1.lon << tmp_point2.lon << myMapBorder[i].lon << i << flag;
            }
        }
        if ( ( (((scanlat-mapMin.lat)/meRouSet.StepLinex)) % 2 ) == 0 ) {
            tmp_append.lon = tmp_point1.lon;//meScaleLon * ( tmp_point1.lon - myBase.lon );
            tmp_append.lat = tmp_point1.lat;//meScaleLat * ( tmp_point1.lat - myBase.lat );
            tmp_append.alt = tmp_point1.alt;
            myRoutePlan.emplace_back(tmp_append);
            tmp_append.lon = tmp_point2.lon;//meScaleLon * ( tmp_point2.lon - myBase.lon );
            tmp_append.lat = tmp_point2.lat;//meScaleLat * ( tmp_point2.lat - myBase.lat );
            tmp_append.alt = tmp_point2.alt;
            myRoutePlan.emplace_back(tmp_append);
//printf("T1Map-RouteZLat step2d1, size=%ld, tmp_append:(%lld, %lld)\n", myRoutePlan.size(), tmp_append.lon, tmp_append.lat);// meWorkline << mapMin.lat << mapMax.lat;
        } else {
            tmp_append.lon = tmp_point2.lon;//meScaleLon * ( tmp_point2.lon - myBase.lon );
            tmp_append.lat = tmp_point2.lat;//meScaleLat * ( tmp_point2.lat - myBase.lat );
            tmp_append.alt = tmp_point2.alt;
            myRoutePlan.emplace_back(tmp_append);
            tmp_append.lon = tmp_point1.lon;//meScaleLon * ( tmp_point1.lon - myBase.lon );
            tmp_append.lat = tmp_point1.lat;//meScaleLat * ( tmp_point1.lat - myBase.lat );
            tmp_append.alt = tmp_point1.alt;
            myRoutePlan.emplace_back(tmp_append);
//printf("T1Map-RouteZLat step2d2, size=%ld, tmp_append:(%lld, %lld)\n", myRoutePlan.size(), tmp_append.lon, tmp_append.lat);// meWorkline << mapMin.lat << mapMax.lat;
        }
    }
//printf("T1Map-RouteZLat end. myRoutePlan.size()=%ld\n", myRoutePlan.size());// meWorkline << mapMin.lat << mapMax.lat;
    return myRoutePlan.size();
}

int T1Map::RouteZLon() { // Z,lon
    MapPoint tmp_point1,tmp_point2,tmp_append;
    int flag = 0;

//printf("T1Map-RouteZLon step1, scan longitude to build the RoutePlan, meWorkline=%d, Min:(%lld, %lld), Max:(%lld, %lld)\n", meWorkline, mapMin.lon, mapMin.lat, mapMax.lon, mapMax.lat);
    tmp_point1 = mapMin;
    tmp_point2 = mapMin;
    //return 0;
    for ( int scanlon=mapMin.lon; scanlon<mapMax.lon; scanlon+=meRouSet.StepLinex ) {
        //std::cout << "T1Map-RouteZLon step2a, scan map longitude. " << mapMin.lon << scanlon-mapMin.lon << tmp_point1.lat << tmp_point2.lat;
        tmp_point1.lon = scanlon;
        tmp_point2.lon = scanlon;
        flag = 0;
        for ( int i=0; i<myMapBorder.size(); i++ ) {
            if ( myMapBorder[i].lon == scanlon ) {  // found at least one point
                if ( flag == 0 ) { // first point in this longitude
                    tmp_point1.lat = myMapBorder[i].lat;
                    tmp_point2.lat = myMapBorder[i].lat;
                    //std::cout << "T1Map-RouteZLon step2b, scan longitude's meWorkline. " << mapMin.lon << scanlon-mapMin.lon << tmp_point1.lat << tmp_point2.lat << myMapRemote[i].lat << i << flag;
                    flag = 1;
                }
                if ( myMapBorder[i].lat < tmp_point1.lat ) tmp_point1.lat = myMapBorder[i].lat;
                if ( myMapBorder[i].lat > tmp_point2.lat ) tmp_point2.lat = myMapBorder[i].lat;
                tmp_point1.alt = i;
                //std::cout << "T1Map-RouteZLon step2c, scan longitude's meWorkline. " << mapMin.lon << scanlon-mapMin.lon << tmp_point1.lat << tmp_point2.lat << myMapBorder[i].lat << i << flag;
            }
        }
        if ( ( (((scanlon-mapMin.lon)/meRouSet.StepLinex)) % 2 ) == 0 ) {
            tmp_append.lon = tmp_point1.lon;//meScaleLon * ( tmp_point1.lon - myBase.lon );
            tmp_append.lat = tmp_point1.lat;//meScaleLat * ( tmp_point1.lat - myBase.lat );
            tmp_append.alt = tmp_point1.alt;
            myRoutePlan.emplace_back(tmp_append);
            tmp_append.lon = tmp_point2.lon;//meScaleLon * ( tmp_point2.lon - myBase.lon );
            tmp_append.lat = tmp_point2.lat;//meScaleLat * ( tmp_point2.lat - myBase.lat );
            tmp_append.alt = tmp_point2.alt;
            myRoutePlan.emplace_back(tmp_append);
//printf("T1Map-RouteZLon step2d1, size=%ld, tmp_append:(%lld, %lld)\n", myRoutePlan.size(), tmp_append.lon, tmp_append.lat);// meWorkline << mapMin.lat << mapMax.lat;
        } else {
            tmp_append.lon = tmp_point2.lon;//meScaleLon * ( tmp_point2.lon - myBase.lon );
            tmp_append.lat = tmp_point2.lat;//meScaleLat * ( tmp_point2.lat - myBase.lat );
            tmp_append.alt = tmp_point2.alt;
            myRoutePlan.emplace_back(tmp_append);
            tmp_append.lon = tmp_point1.lon;//meScaleLon * ( tmp_point1.lon - myBase.lon );
            tmp_append.lat = tmp_point1.lat;//meScaleLat * ( tmp_point1.lat - myBase.lat );
            tmp_append.alt = tmp_point1.alt;
            myRoutePlan.emplace_back(tmp_append);
//printf("T1Map-RouteZLon step2d2, size=%ld, tmp_append:(%lld, %lld)\n", myRoutePlan.size(), tmp_append.lon, tmp_append.lat);// meWorkline << mapMin.lat << mapMax.lat;
        }
    }

    if ( (myRoutePlan.size()/2)%2 == 1 ) {    // longitude Z scan must be even lines
        tmp_append.lon = tmp_point2.lon;//meScaleLon * ( tmp_point2.lon - myBase.lon );
        tmp_append.lat = tmp_point2.lat;//meScaleLat * ( tmp_point2.lat - myBase.lat );
        tmp_append.alt = tmp_point2.alt;
        myRoutePlan.emplace_back(tmp_append);
        tmp_append.lon = tmp_point1.lon;//meScaleLon * ( tmp_point1.lon - myBase.lon );
        tmp_append.lat = tmp_point1.lat;//meScaleLat * ( tmp_point1.lat - myBase.lat );
        tmp_append.alt = tmp_point1.alt;
        myRoutePlan.emplace_back(tmp_append);
//printf("T1Map-RouteZLon step3a, size=%ld, tmp_append:(%lld, %lld)\n", myRoutePlan.size(), tmp_append.lon, tmp_append.lat);// meWorkline << mapMin.lat << mapMax.lat;
    }

//printf("T1Map-RouteZLon end. myRoutePlan.size()=%ld\n", myRoutePlan.size());// meWorkline << mapMin.lat << mapMax.lat;
    return myRoutePlan.size();
}

/* Insert points between RTK Trailed points
 */
int T1Map::GetBorder() {
    int ret = GetRemote();
    if ( ret <= 1 ) return ret; // at least must have two points
//printf("T1Map-GetBorder begin. ret=%d\n", ret);

    int dlon,dlat,dmax,insertlon,insertlat;
    MapPoint tmp_point;

    myMapBorder.clear();
    for ( int i=2; i<myMapRemote.size(); i++ ) {
        tmp_point = myMapRemote[i-1];
        myMapBorder.emplace_back(tmp_point);
        //printf("T1Map-GetBorder loop(%d-%d), point(%lld,%lld)-(%lld,%lld)\n", ret, i, myMapRemote[i-1].lon, myMapRemote[i-1].lat, myMapRemote[i].lon, myMapRemote[i].lat);
        dlon = myMapRemote[i].lon - myMapRemote[i-1].lon;
        dlat = myMapRemote[i].lat - myMapRemote[i-1].lat;
        dmax = abs(dlon)>abs(dlat) ? abs(dlon) : abs(dlat);
        //std::cout << "T1Map-GetBorder step2, presetting: " << dlon << dlat << dmax;// << steplon << steplat;
        if ( dmax > PRECISION/1000 ) continue;
        for ( int j=1; j<dmax; j++ ){
            insertlon = myMapRemote[i-1].lon + (j*dlon/dmax);
            insertlat = myMapRemote[i-1].lat + (j*dlat/dmax);
            //std::cout << "T1Map-GetBorder step3, insert: " << i << j << insertlon << insertlat;
            tmp_point.lon = insertlon;
            tmp_point.lat = insertlat;
            myMapBorder.emplace_back(tmp_point);
        }
    }

printf("T1Map-GetBorder end. myMapBorder.size()=%ld\n", myMapBorder.size());
    return myMapBorder.size();
}

int T1Map::GetRemote() {
    int debug=1;
    char filename[100];
    //std::cout << "\nT1Map-GetRemote step1: read TrailBorder.map" << std::endl;
    sprintf(filename,"../Map/MapDefault.maap");
    std::ifstream filein(filename);
    if (!filein.is_open()) {
        T1Utils::LogBug("T1Map-GetRemote, Can not open MapDefault.maap");
        sprintf(filename,"./upt/TrailBorder100x100.map");
        std::cerr <<"T1Map-GetRemote, try default:[" << filename << "]" << std::endl;
        filein.open(filename);
        if (!filein.is_open()) {
            T1Utils::LogBug("T1Map-GetRemote, Can not open default ./upt/TrailBorder100x100.map");
            return -1;
        }    
    }

    //std::cout << "\nT1Map-GetRemote step2: begin loop reading." << std::endl;

    std::string line;
    std::vector<std::string> list1;

    int count = 0;
    MapPoint tmp_point;
    myMapRemote.clear();

    while ( std::getline(filein,line) ) {
        //count ++;
        if ( line[0] == '#' ) continue;
        list1 = T1Utils::split(line,',');
//printf("T1Map-GetRemote loop %d reading: %ld[%s]\n", count, list1.size(), line.c_str());
        if ( list1.size() < 6 ) continue;
        tmp_point.lon = std::stol(list1[0]);
        tmp_point.lat = std::stol(list1[1]);
        tmp_point.alt = std::stoi(list1[2]);
        tmp_point.fixed = std::stoi(list1[3]);
        tmp_point.sate = std::stoi(list1[4]);
        tmp_point.roll = std::stoi(list1[5]);

        //if ( tmp_point.lon == 3872440 )
            //std::cout << "T1Map-GetRemote step2, reading: " << count << line << tmp_point.lon << tmp_point.lat;
        if ( tmp_point.roll==-1 || tmp_point.roll==-2 ) {
            ReadForbidden(tmp_point);
            continue;
        }
        if ( tmp_point.roll == -101 || tmp_point.roll == -102 ) {
            ReadAreaPassOnly(tmp_point);
            continue;
        }
        if ( tmp_point.roll == -1000 ) {
            ReadAprilTagPos(tmp_point);
            continue;
        }
        myMapRemote.emplace_back(tmp_point);
        count++;
    }
    //printf("T1Map-GetRemote step3, count=%d, myMapRemote.size()=%ld, baselon:(%lld, %lld)\n", count, myMapRemote.size(), myBase.lon, myBase.lat);
    filein.close();

    myBase.lon = -1;
    myBase.lat = -1;
    tmp_point.lon = 0;
    tmp_point.lat = 0;
    if ( count == 0 ) { // 0 point
        myMapRemote.emplace_back(myBase); // -1,-1 means don't use RTK
        myMapRemote.emplace_back(tmp_point);
        myMapRemote.emplace_back(tmp_point);
    } else if ( count == 1 ) { //
        myBase = myMapRemote[0];
        myMapRemote.emplace_back(tmp_point);
        myMapRemote.emplace_back(tmp_point);
    } else { //if ( count == 2 ) { >=2 add the 0,0 at the tail
        myMapRemote.emplace_back(tmp_point);
    }

    //std::cout << "T1Map-GetRemotek step4, get max & min: " << WorkStatus6;
    // get base point
    myBase = myMapRemote[0];    myBase.yaw = 0;
    // Determine if it is the default map
    isDefaultMap = (myBase.lon == -1 && myBase.lat == -1);
    if (isDefaultMap) {
        if (debug>0) printf("T1Map-GetRemote: Default map detected (base=-1,-1), isDefaultMap=true\n");
    } else {
        if (debug>0) printf("T1Map-GetRemote: RTK map detected (base=%lld,%lld), isDefaultMap=false\n", myBase.lon, myBase.lat);
    }
    mapMax = myMapRemote[1];
    mapMin = myMapRemote[1];
    //std::cout << "\nT1Map-GetRemote step5: " << myMapRemote[1].lon << myMapRemote[1].lat << mapMax.lon << mapMax.lat << mapMin.lon << mapMin.lat;
    for ( int i=2; i<myMapRemote.size(); i++ ) {
        //std::cout << "T1Map-GetRemote step5a, scan map. " << i << myMapRemote[i].lat << myMapRemote[i].lon;
        if ( myMapRemote[i].lat > mapMax.lat ) mapMax.lat=myMapRemote[i].lat;
        if ( myMapRemote[i].lon > mapMax.lon ) mapMax.lon=myMapRemote[i].lon;
        if ( myMapRemote[i].lat < mapMin.lat ) mapMin.lat=myMapRemote[i].lat;
        if ( myMapRemote[i].lon < mapMin.lon ) mapMin.lon=myMapRemote[i].lon;
    }

printf("T1Map-GetRemote end. myMapRemote.size()=%ld, Base:(%lld, %lld), Max:(%lld, %lld), Min:(%lld, %lld)\n", myMapRemote.size(), myBase.lon, myBase.lat, mapMax.lon, mapMax.lat, mapMin.lon, mapMin.lat);
    return myMapRemote.size();
}

// this function do nothing, it's a virtual for child class to override
void T1Map::ReadForbidden(MapPoint point) {
}

// this function do nothing, it's a virtual for child class to override
void T1Map::ReadAreaPassOnly(MapPoint point) {
}

// this function do nothing, it's a virtual for child class to override
void T1Map::ReadAprilTagPos(MapPoint point) {
}

// this function do nothing, it's a virtual for child class to override
int T1Map::SetAprilTagChrgStation(int stationlength) {
    return 0;
}

/***************************************************/
// 1. MapTrail file (point-marking file) naming rule: TrailYYYYMMDD-HH:MM:SS.map
//    - Timestamp = time of 2nd 't' key press (end time of marking)
//    - Manually drawn files: *.map suffix
// 2. Source file for remote-controlled boundary marking: TrailRomote.rtk
// 3. MapSelect.dat: stores filename of the selected map file
// 4. MapDefault.maap: copy of selected map file (formerly Defaultmap.map)
//    - Suffix changed to .maap for easy differentiation from .map files
// 5. MapChecksum.dat: stores checksum calculated by:
//    - Step 1: Sum ASCII values of each character in all *.map filenames (Map directory)
//    - Step 2: Add ASCII sum of selected map filename (added twice total)
//    - Checksum changes if: *.map files modified OR selected map changed
//    - Transmission logic:
//      - Send checksum to APP with heartbeat data
//      - Mobile terminal recalculates checksum independently
//      - If X3 ↔ mobile checksum mismatch: mobile sends command to X3 to upload:
//        - Map file directory list
//        - Selected map filename
//        - Content of selected map file
/** write the ../Map/MapSelect.dat
 * 2026.02.20: created by Leo
 */
int T1Map::MapSelectWrite(const char * mapfilename) {
    //printf("T1Map-MapSelectWrite begin, [%s]\n", mapfilename);
    char tmpstr[200];
    sprintf(tmpstr,"../Map/MapSelect.dat");
    std::ofstream fileout2(tmpstr);
    if (!fileout2.is_open()) {
        T1Utils::LogBug("T1Map-MapSelectWrite, Can not open MapSelect.dat");
        GetRoute(0);
        return -1;
    }
    fileout2 << mapfilename;
    fileout2.close();
    printf("T1Map-MapSelectWrite end, [%s]\n", mapfilename);
    return 0;
}

/** read the ../Map/MapSelect.dat
 * 2026.02.20: created by Leo
 */
std::string T1Map::MapSelectRead() {
    std::string line="";
    char tmpstr[200];
    sprintf(tmpstr,"../Map/MapSelect.dat");
    std::ifstream filein(tmpstr);
    if (!filein.is_open()) {
        T1Utils::LogBug("T1Map-MapSelectRead, Can not open ../Map/MapSelect.dat");
        return line;
    }
    while ( std::getline(filein,line) ) {
        //for (char c : line) {
        //    checksum += static_cast<int>(c); // Accumulate ASCII value with filename of the selected map
        //}        
    }
    filein.close();
    printf("T1Map-MapSelectRead end. [%s]\n", line.c_str());
    return line;
}

/**
 * @brief Traverse directory using Linux system calls (no C++17 required), collect .map files
 * @param dir_path Target directory path
 * @return Sorted list of .map file absolute paths
 * 2026.02.14: created by Leo
 */
std::vector<std::string> T1Map::MapGetList() {//const std::string& dir_path) {
    std::string dir_path = "../Map";
    std::vector<std::string> map_filenames;
    DIR* dir = opendir(dir_path.c_str());
    //DIR* dir = opendir("../Map");
    if (!dir) {
        std::cerr << "T1Map::MapGetList, Failed to open directory: " << dir_path << std::endl;
        return map_filenames;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        // Skip . and .. directories
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }
        // Only process regular files with .map suffix
        if (entry->d_type == DT_REG) {
            std::string filename = entry->d_name;
            if (filename.size() >= 4 && filename.substr(filename.size()-4) == ".map") {
                // Concatenate absolute path (simple implementation)
                //std::string abs_path = dir_path + "/" + filename;
                map_filenames.push_back(filename);//abs_path);
            }
        }
    }
    closedir(dir);
    // Sort to ensure consistent calculation order
    std::sort(map_filenames.begin(), map_filenames.end());
    return map_filenames;
}

/**
 * @brief Calculate ASCII sum checksum for filename list (replace hash/parity check)
 * Logic: Traverse each character of each filename, accumulate ASCII values, return total sum
 * @param filenames Sorted list of .map filenames
 * @return Total ASCII sum of all filename characters
 * 2026.02.14: created by Leo
 */
long long T1Map::MapGetChecksum() {//const std::vector<std::string>& filenames) {
    long long checksum = 0; // Use long long to avoid overflow

    auto mapfiles = MapGetList();
    for (const auto& filename : mapfiles) {
        //printf("T1Map-MapGetChecksum loop:[%s]\n", filename.c_str());
        for (char c : filename) {
            checksum += static_cast<int>(c); // Accumulate ASCII value of each character
        }
    }

    std::string line = MapSelectRead();
    //printf("T1Map-MapGetChecksum select:[%s]\n", line.c_str());
    for (char c : line) {
        checksum += static_cast<int>(c); // Accumulate ASCII value with filename of the selected map
    }

    char tmpstr[200];
    sprintf(tmpstr,"../Map/MapChecksum.dat");
    std::ofstream fileout(tmpstr);
    if (!fileout.is_open()) {
        T1Utils::LogBug("T1Map-MapGetChecksum, Can not open log TrailRemote.rtk");
        return -1;
    }
    sprintf(tmpstr, "%lld", checksum);
    fileout << tmpstr << std::endl;
    fileout.close();
    
    printf("T1Map-MapGetChecksum end, checksum=[%s]\n", tmpstr);
    return checksum;
}

/** simulate the APP's command string
 * select map and set default map, 
 * 2026.02.20: created by Leo
 */ 
void T1Map::MapSelect(const char * mapname) {
    char srcfile[200];
    sprintf(srcfile,"../Map/%s", mapname);
    printf("T1Map-MapSelect begin, map[%s].\n", srcfile);
    if (!T1Utils::copyFile(srcfile, "../Map/MapDefault.maap")) {
        T1Utils::LogBug("T1Map-MapSelect, failed to copy TrailBorder.map to MapDefault.maap");
    }
    MapSelectWrite(mapname);
    //MapGetChecksum(); // must be called in T1All.cpp so to syn T1All.meMapChecksum
    GetRoute(0);
    printf("T1Map-MapSelect end, map[%s].\n", srcfile);
}
