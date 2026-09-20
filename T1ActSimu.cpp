#include "T1ActSimu.hpp"
//#include <vector>
//#include <string>
#include <cstring>
#include <iostream>
#include <fstream>
//#include <cstdio>
#include <math.h>

T1ActSimu::T1ActSimu() {
}

T1ActSimu::~T1ActSimu() {
    
}

/* In one line of the map travarsal, simulate the Mower goto the target point step by step
 * return==0, successfully finished the line work,   return>0: steps worked in this line
 * return<0: failed in this line, but not used now
 */
int T1ActSimu::WorkLine(MapPoint end) {
    int debug=0;
//int T1ActSimu::WorkLine2(MapPoint begin, MapPoint end) {
    //printf("T1ActSimu-WorkLine begin: line=%d, step=%d, begin:(%lld,%lld,%d), end:(%lld,%lld,%d)\n", meWorkline, meWorkstep, begin.lon, begin.lat, begin.alt, end.lon, end.lat, end.alt);
    static MapPoint distance, begin;
    static int stepmax = 0, distanceLine = 0;
    static int flag=0; // flag=0: should rotate, flag=1: should move
    int tmp_yaw=0, tmp_go=0, stepLINE;
    stepLINE = meFact.StepLineg * 0.5;  // STEPLINE;//5;

    if ( meWorkstep == 0 ) { // initialize of this Work line
        //printf("T1ActSimu-WorkLine step1a, line presetting: step=%d, distance:(%lld,%lld,%d)\n", meWorkstep, distance.lat, distance.lon, distanceLine);
        distance.lon = end.lon - myMower6.fx;
        distance.lat = end.lat - myMower6.fy;
        distanceLine = sqrt (  distance.lon*distance.lon + distance.lat*distance.lat );
        //printf("T1ActSimu-WorkLine step1b, line presetting: step=%d, distance:(%lld,%lld,%d)\n", meWorkstep, distance.lat, distance.lon, distanceLine);
        if ( distanceLine == 0 ) {
            //printf("T1ActSimu-WorkLine step1b, line presetting: step=%d, distance:(%lld,%lld,%d)\n", meWorkstep, distance.lat, distance.lon, distanceLine);
            return 0;
        }
        stepmax = distanceLine / stepLINE;

        //myMower6.fx = begin.lon;
        //myMower6.fy = begin.lat;
        meWorkstep = 1;
        flag = 0; // judge rotation or walk
        //printf("T1ActSimu-WorkLine step1c, line presetting: step=%d, distance:(%lld,%lld,%d)\n", meWorkstep, distance.lat, distance.lon, distanceLine);
    }
    if ( meWorkstep < stepmax+1 ) { // walk step by step
        distance.lon = end.lon - myMower6.fx;
        distance.lat = end.lat - myMower6.fy;
        distance.alt = sqrt (  distance.lon*distance.lon + distance.lat*distance.lat );
        //printf("T1ActSimu-WorkLine step2, (%d,%d,%d), distance:(%lld,%lld,%d)\n", meWorklength, meWorkline, meWorkstep, distance.lat, distance.lon, distanceLine);
        if ( distance.alt == 0 ) { // distance.alt is not the altitude, just used to remember the distance
            //printf("T1ActSimu-WorkLine step2-a, alt==0.     step=%d, distance:(%lld,%lld), distanceLine=%d\n", meWorkstep, distance.lat, distance.lon, distanceLine);
            return 0;
        }
        if ( flag == 0 ) { // rotation
            tmp_yaw = myMower6.yaw;
            if ( distance.lat > 0 )
                myMower6.yaw = asin(((float)distance.lon)/distance.alt) * 180 / 3.14159265;
            else
                myMower6.yaw = 180 - asin(((float)distance.lon)/distance.alt) * 180 / 3.14159265;
            tmp_yaw = myMower6.yaw - tmp_yaw;
            flag = 1;
            //printf("T1ActSimu-WorkLine step2b, rotate: %d, flag=%d, %d, %d\n", meWorkstep, flag, myMower6.yaw, tmp_yaw);
        } else { // walk
            if ( distance.alt < stepLINE )
                tmp_go = distance.alt;
            else
                tmp_go = stepLINE;
            myMower6.fx += tmp_go * sin(myMower6.yaw * 3.14159265 /180);//= begin.lon + meWorkstep*(end.lon-begin.lon)/stepmax;//+= tmp_go * (double)distance.lon / distance.alt;
            myMower6.fy += tmp_go * cos(myMower6.yaw * 3.14159265 /180);//= begin.lat + meWorkstep*(end.lat-begin.lat)/stepmax;//+= tmp_go * (double)distance.lat / distance.alt;

            flag = 0;
            //printf("T1ActSimu-WorkLine step2c, step: %d, flag=%d, %d, %d, %d %d, myMower6:(%.1f,%.1f,%d)\n", meWorkstep, flag, tmp_go, distance.alt, stepLINE, STEPLINE, myMower6.fx, myMower6.fy, myMower6.yaw);
            meWorkstep++;
        }
    } else {
        if (debug>0) printf("T1ActSimu-WorkLine step3: step=%d, %d, %d\n", meWorkstep, meWorkstep*STEPLINE, distance.alt);
        meWorkstep = 0;
    }
    //printf("T1ActSimu-WorkLine end: line=%d, step=%d\n", meWorkline, meWorkstep);
    if ( FlagStop>=5 && FlagStop!=8 ) {
        printf("T1ActSimu-WorkLine, FlagStop=%d\n",FlagStop);
        int ret = -1 * FlagStop;
        FlagStop = 0;
        return ret;
    }
    return meWorkstep;
}

/* only show the moving on screen
 * return==1: Work,  return<0: finished work,
 */
int T1ActSimu::Work() {
    //printf("T1ActSimu-Work begin: WS6=%d, (%d,%d,%d), Min(%lld,%lld), Max(%lld,%lld)\n", WorkStatusSimu, meWorklength, meWorkline, meWorkstep, pMap->GetPoint('i',0).lon, pMap->GetPoint('i',0).lat, pMap->GetPoint('a',0).lon, pMap->GetPoint('a',0).lat);
    MapPoint tmpPoint;
    if ( WorkStatusSimu < 0 || FlagPause == 1) {
        ;
    } else if ( WorkStatusSimu == 0 ) {
        //printf("\n\n\n========== T1ActSimu-Work, WS6=0 =============\n");
        pMap->SetRouSet(meFact.StepLinex,meFact.StepLiney);
        pMap->GetRoute(0);
        meWorklength = pMap->GetLength('P'); meWorkline = 0; meWorkstep = 0;
        WorkStatusSimu = 1;
        myMower6.fx = 0;  myMower6.fy = 0;
        //printf("T1ActSimu-Work() step1: (%d,%d,%d), WS6=%d, Min:(%lld,%lld), Max:(%lld,%lld)\n", meWorklength, meWorkline, meWorkstep, WorkStatusSimu, pMap->GetPoint('i',0).lon, pMap->GetPoint('i',0).lat, pMap->GetPoint('a',0).lon, pMap->GetPoint('a',0).lat);
    } else { 
        //printf("T1ActSimu-Work() step2: (%d,%d,%d), RSW=%d, Min:(%lld,%lld), Max:(%lld,%lld)\n", meWorklength, meWorkline, meWorkstep, WorkStatusSimu, pMap->GetPoint('i',0).lon, pMap->GetPoint('i',0).lat, pMap->GetPoint('a',0).lon, pMap->GetPoint('a',0).lat);
        if ( meWorkline >= meWorklength ) {
            //printf("T1ActSimu-Work() step3, scaned all lines: [%d,meWorkline=(%d,%d)], RSW=%d\n", meWorklength, meWorkline, meWorkstep, WorkStatusSimu);
            WorkStatusSimu = 0; // always loop
            //WorkStatusSimu = -1; // just do once
        } else {
            int ret = WorkLine(pMap->GetPoint('P',meWorkline));
            //int ret = WorkLine2(pMap->GetPoint('P',meWorkline),pMap->GetPoint('P',meWorkline+1));
            if ( ret == 0 ) {
                meWorkline++;
                printf("T1ActSimu-Work() step2, scaned one lines: (%d,%d,%d), RSW=%d, ", meWorklength, meWorkline, meWorkstep, WorkStatusSimu);
                printf("Target:(%lld,%lld)\n", pMap->GetPoint('P',meWorkline-1).lon, pMap->GetPoint('P',meWorkline-1).lat);
            }
        }
    }
    //printf("T1ActSimu-Work() end: (%d,%d,%d), WS6=%d, Min:(%lld,%lld), Max:(%lld,%lld)\n", meWorklength, meWorkline, meWorkstep, WorkStatusSimu, mapMin.lon, mapMin.lat, mapMax.lon, mapMax.lat);
    return WorkStatusSimu;
}

/* return==1: working,  return<0: finished work,
 */
int T1ActSimu::WorkAll() {
    //printf("T1ActSimu-WorkAll, begin.\n");
    return Work();
}




