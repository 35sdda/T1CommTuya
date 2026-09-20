#include "T1ActSimuAll.hpp"
//#include <vector>
//#include <string>
#include <cstring>
#include <cstdio>
#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
//#include <cstdio>
#include <math.h>
#include "T1Utils.hpp"

T1ActSimuAll::T1ActSimuAll() {
}

T1ActSimuAll::~T1ActSimuAll() {
    
}

/** save the current gridmap before gohome()
 * 2026.02.27: created by Leo
 */
void T1ActSimuAll::GoHomeGridMapSave() {
    printf("T1ActSimuAll-GoHomeSaveGridMap begin.\n");
}

/** read the gridmap saved when gohome()
 * 2026.02.27: created by Leo
 */
void T1ActSimuAll::GoHomeGridMapRead() {
    printf("T1ActSimuAll-GoHomeReadGridMap begin.\n");
}

/** clear the saved point when gohome()
 * first character is 0 means no point saved, 1 means point saved
 * 2026.02.27: created by Leo
 */
int T1ActSimuAll::GoHomeClearPoint() {
    printf("T1ActSimuAll-GoHomeClearPoint begin.\n");
    char tmpstr[200];
    sprintf(tmpstr,"../Tsk/GoHomePoint.dat");
    std::ofstream fileout(tmpstr);
    if (!fileout.is_open()) {
        T1Utils::LogBug("T1ActSimuAll-GoHomeClearPoint, Can not open ../Tsk/GoHomePoint.dat");
        return -1;
    }
    fileout << "0,nomap,0,0,0,0,0,0" << std::endl;
    fileout.close();
    return 0;
}

/** save the current point before gohome()
 * 2026.02.27: created by Leo
 */
int T1ActSimuAll::GoHomeSavePoint() {
    printf("T1ActSimuAll-GoHomeSavePoint begin.\n");
    char tmpstr[200];
    sprintf(tmpstr,"../Tsk/GoHomePoint.dat");
    std::ofstream fileout(tmpstr);
    if (!fileout.is_open()) {
        T1Utils::LogBug("T1ActSimuAll-GoHomeSavePoint, Can not open ../Tsk/GoHomePoint.dat");
        return -1;
    }
    fileout << "1," << pMap->MapSelectRead() << ",";
    fileout << ((T1Map2 *)pMap)->GetLength('x') << "," << ((T1Map2 *)pMap)->GetLength('y') << ",";
    fileout << myMower6.fx << "," << myMower6.fy << "," << meWorkline << "," << flagReverse << "," << std::endl;
    fileout.close();
    ((T1Map2 *)pMap)->GridMapSave("../Tsk/GoHomeGridMap.dat");
    return 0;
}

/** read the saved point when gohome()
 * return:  0,no saved pint         1,get saved point
 * 2026.02.27: created by Leo
 */
int T1ActSimuAll::GoHomeReadPoint() {
    //printf("T1ActSimuAll-GoHomeReadPoint begin.\n");
    int flagSaved = 0;
    char tmpstr[200];
    sprintf(tmpstr,"../Tsk/GoHomePoint.dat");
    std::ifstream filein(tmpstr);
    if (!filein.is_open()) {
        T1Utils::LogBug("T1ActSimuAll-GoHomeReadPoint, Can not open ../Tsk/GoHomePoint.dat");
        return flagSaved;
    }
    
    std::string line;
    std::vector<std::string> list1;
    while ( std::getline(filein,line) ) {
        list1 = T1Utils::split(line,',');
        //printf("T1ActSimuAll-GoHomeReadPoint step1, length=%ld.\n", list1.size());
        if ( list1.size() != 8 ) { // if you add some fields, notice the GoHomeClearPoint();
            break;
        }
        if ( line[0] == '1' ) {
            std::string mapselect = pMap->MapSelectRead();
            //printf("T1ActSimuAll-GoHomeReadPoint step2, [%s],(%s).\n", list1[1].c_str(), mapselect.c_str());
            if ( list1[1] == mapselect  // select map changed
                && ((T1Map2 *)pMap)->GetLength('x') == std::stoi(list1[2]) // grid map size error
                && ((T1Map2 *)pMap)->GetLength('y') == std::stoi(list1[3]) ) {
                printf("T1ActSimuAll-GoHomeReadPoint step3, saved map restored.\n");
                flagSaved = 1;
                GoHomePoint.fx = std::stod(list1[4]);
                GoHomePoint.fy = std::stod(list1[5]);
                meWorkline = std::stoi(list1[6]);
                flagReverse = std::stoi(list1[7]);
                ((T1Map2 *)pMap)->GridMapRead("../Tsk/GoHomeGridMap.dat");
                break; // only getline 1 line
            } else {
                printf("T1ActSimuAll-GoHomeReadPoint step4, saved map don't match.\n");
            }
        }
    }
    filein.close();
    //printf("T1ActSimuAll-GoHomeReadPoint end. read(%s)\n", line.c_str());
    printf("T1ActSimuAll-GoHomeReadPoint end. read(%d,%s,%.1f,%.1f,%d,%d,%d,%d)\n", flagSaved, list1[1].c_str(), GoHomePoint.fx, GoHomePoint.fy, meWorkline, flagReverse, meWorkline, flagReverse);
    return flagSaved;
}

int T1ActSimuAll::GoHome() {
    if ( WorkStatusGH < 0 || FlagPause == 1 ) {
        return WorkStatusGH;
    } else if ( WorkStatusGH == 0 ) {
        printf("T1ActSimuAll-GoHome[0]: prepare to go home using A* path. myMower6(%.1f,%.1f)\n", myMower6.fx, myMower6.fy);
        RoboAct('g', 0);
        int endX = 0, endY = 0;
        if (((T1Map2 *)pMap)->findAstarToPoint((int)myMower6.fx, (int)myMower6.fy, endX, endY) < 0) {
            T1Utils::LogBug("T1ActSimuAll-GoHome[0]: error, findAstarToPoint() failed.\n");
            WorkStatusGH = -1;
        } else {
            //printf("T1ActSimuAll-GoHome[0]: get the RoutePlan2.length=%d\n", pMap->GetLength('2'));
            WorkStatusSimu = 0; // WorkStatus to control Work2();
            WorkStatusGH = 1;
        }
    } else if (WorkStatusGH == 1) {
        int retWork = Work2();
        if (retWork < 0) {
            printf("T1ActSimuAll-GoHome[1]: work2() completed, reach (0,0).\n");
            WorkStatusGH = -1;
        }
    } else {
        char bugMsg[128];
        snprintf(bugMsg, sizeof(bugMsg), "T1ActSimuAll-GoHome: ERROR unknown WorkStatusGH=%d", WorkStatusGH);
        T1Utils::LogBug(bugMsg);
        WorkStatusGH = -1;
        RoboAct('g', 0);
    }
    return WorkStatusGH;
}

/* Cut along the border
 * 2026.04.13: created by Leo
 */
int T1ActSimuAll::GoBorder() {
    static MapPoint nextPoint;
    if ( WorkStatusGB<0 || FlagPause==1 ) {
        return WorkStatusGB;
    } else if ( WorkStatusGB==0 ) {
        printf("T1ActSimuAll-GoBorder step0: prepare to go home using A* path. myMower6(%.1f,%.1f)\n", myMower6.fx, myMower6.fy);
        RoboAct('g', 0);
        if (((T1Map2 *)pMap)->GetRouteBorder() < 0) {
            T1Utils::LogBug("T1ActSimuAll-GoBorder[0]: error, findAstarToPoint() failed.\n");
            WorkStatusGB = -1;
        } else {
            //printf("T1ActSimuAll-GoHome[0]: get the RoutePlan2.length=%d\n", pMap->GetLength('2'));
            WorkStatusGB = 1;
        }
    } else if ( WorkStatusGB==1 ) {
        meWorklength = pMap->GetLength('P'); meWorkline = 0; meWorkstep = 0;
        //myMower6.fx = 0;  myMower6.fy = 0;
        //flagReverse = 0;
        MapPoint nextPoint = pMap->GetPoint('P',meWorkline);
        myMower6.fx = nextPoint.lon;  myMower6.fy = nextPoint.lat;
        meWorkline++;
        WorkStatusGB = 2;
        printf("T1ActSimuAll-GoBorder step1: prepare to workall lines. workline=%d\n", meWorkline );
    } else if ( WorkStatusGB==2 ) {
        nextPoint = pMap->GetPoint('P',meWorkline);
        ((T1Map2 *)pMap)->findAstarToPoint(myMower6.fx, myMower6.fy, nextPoint.lon, nextPoint.lat);
        printf("T1ActSimuAll-GoBorder step2: prepare find Astar route. workline=%d, now=(%.f,%.f), target=(%lld,%lld)\n",
            meWorkline, myMower6.fx,myMower6.fy, nextPoint.lon, nextPoint.lat );
        WorkStatusSimu = 0; // WorkStatus to control Work2();
        WorkStatusGB = 3;
    } else if ( WorkStatusGB==3 ) {
        int retWork = Work2();
        if (retWork < 0) {
            printf("T1ActSimuAll-GoBorder step3: work2() completed. workline=%d \n", meWorkline);
            myMower6.fx = nextPoint.lon;  myMower6.fy = nextPoint.lat;
            meWorkline++;
            if ( meWorkline>=meWorklength )
                WorkStatusGB = -1;
            else
                WorkStatusGB = 2;
        }
    } else {
        char bugMsg[128];
        snprintf(bugMsg, sizeof(bugMsg), "T1ActSimuAll-GoBorder: ERROR unknown WorkStatusGH=%d", WorkStatusGH);
        T1Utils::LogBug(bugMsg);
        WorkStatusGB = -1;
        RoboAct('g', 0);
    }
    return WorkStatusGB;
}

/** work on RoutePlan2, and use the T1Map2 class
 * return==1: Work,  return<0: finished work,
 */
int T1ActSimuAll::Work2() {
    int debug=0;
    //printf("T1ActSimuAll-Work2 begin: WS6=%d, (%d,%d,%d), Min(%lld,%lld), Max(%lld,%lld)\n", WorkStatusSimu, meWorklength, meWorkline, meWorkstep, pMap->GetPoint('i',0).lon, pMap->GetPoint('i',0).lat, pMap->GetPoint('a',0).lon, pMap->GetPoint('a',0).lat);
    MapPoint tmpPoint;
    if ( WorkStatusSimu < 0 || FlagPause == 1) {
        ;
    } else if ( WorkStatusSimu == 0 ) {
        //pMap->SetRouSet(meFact.StepLinex,meFact.StepLiney);
        //pMap->GetRoute(0);
        meWorklength2 = pMap->GetLength('2'); meWorkline2 = 0; meWorkstep = 0;
        WorkStatusSimu = 1;
        //myMower6.fx = 0;  myMower6.fy = 0;
        if (debug>0) printf("T1ActSimuAll-Work2 step1: (%d,%d,%d), WS6=%d, Min:(%lld,%lld), Max:(%lld,%lld)\n", meWorklength2, meWorkline2, meWorkstep, WorkStatusSimu, pMap->GetPoint('i',0).lon, pMap->GetPoint('i',0).lat, pMap->GetPoint('a',0).lon, pMap->GetPoint('a',0).lat);
    } else {
        //printf("T1ActSimuAll-Work2 step2: (%d,%d,%d), RSW=%d, Min:(%lld,%lld), Max:(%lld,%lld)\n", meWorklength, meWorkline, meWorkstep, WorkStatusSimu, pMap->GetPoint('i',0).lon, pMap->GetPoint('i',0).lat, pMap->GetPoint('a',0).lon, pMap->GetPoint('a',0).lat);
        if ( meWorkline2 >= meWorklength2 ) {
            //printf("T1ActSimuAll-Work2 step3, scaned all lines: [%d,meWorkline=(%d,%d)], RSW=%d\n", meWorklength, meWorkline, meWorkstep, WorkStatusSimu);
            WorkStatusSimu = -1; // just do once
        } else {
            int ret = WorkLine(((T1Map2 *)pMap)->GetPoint2('2',meWorkline2));
            //printf("T1ActSimuAll-Work2 step4: (%d,%d,%d), WS6=%d, ret=%d\n", meWorklength2, meWorkline2, meWorkstep, WorkStatusSimu, ret);
            if ( ret > 0 ) {
                SetCutGrid();
            } else if ( ret == 0 ) {
                meWorkline2++;
                if (debug>0) printf("T1ActSimuAll-Work2 step6, scaned one lines: (%d,%d,%d), RSW=%d, ", meWorklength2, meWorkline2, meWorkstep, WorkStatusSimu);
                if (debug>0) printf("Target:(%lld,%lld)\n", ((T1Map2 *)pMap)->GetPoint2('2',meWorkline2-1).lon, ((T1Map2 *)pMap)->GetPoint2('2',meWorkline2-1).lat);
            } else if ( ret==-8 ) { // real obstacle detected
                printf("T1ActSimuAll-Work2 step7, obastacle detected. ret=%d\n", ret);
                if ( SetObst(mySgbmWall)==1 ) { // set the real obstacle to gridmap, and re-plan all route lines
                    //CutEnd.lon = myMower6.fx; CutEnd.lat = myMower6.fy;
                    //SetCutLine(CutBegin,CutEnd);
                    //WorkStatusSimuAll = 100;
                } else { // continue;
                }
            } else if ( ret==-5 || ret==-9 ) { // simulated obstacle detected
                printf("T1ActSimuAll-Work2 step8, obastacle detected. ret=%d\n", ret);
                //CutEnd.lon = myMower6.fx; CutEnd.lat = myMower6.fy;
                //SetCutLine(CutBegin,CutEnd);
                SetObstSimu(ret);
                //WorkStatusSimuAll = 100;
            } else {
                printf("T1ActSimuAll-Work2 step4, unknown WorkLine return(%d)\n", ret); 
            }
        }
    }
    //printf("T1ActSimuAll-Work2 end: (%d,%d,%d), WS6=%d, Min:(%lld,%lld), Max:(%lld,%lld)\n", meWorklength, meWorkline, meWorkstep, WorkStatusSimu, mapMin.lon, mapMin.lat, mapMax.lon, mapMax.lat);
    return WorkStatusSimu;
}

/** return==1: working,  return<0: finished work,
 * this include find Astar path, Astar walk and the RoutePlanLine walk
 */
int T1ActSimuAll::WorkAll() {
    int ret;
    static int count=0;
    static MapPoint CutBegin, CutEnd;
    //printf("T1ActSimuAll-WorkAll begin, FlagGH=%d.\n", FlagGoHome);
    if ( WorkStatusSimuAll < 0 ) { // Stopped work
        ;
    } else if ( FlagGoHome == 1 ) { // save the point, and gohome now
        //printf("T1ActSimuAll-WorkAll GH=%d.\n", WorkStatusGH);
        if ( WorkStatusGH <0 ) WorkStatusSimuAll = 71;
        FlagGoHome = 0;
    } else if ( FlagPause == 1 ) { // pause work
        //printf("T1ActSimuAll-WorkAll pause=%d.\n", FlagPause);
        ;
    } else if ( WorkStatusSimuAll == 0 ) {
        printf("\n\n\n========== T1ActSimuAll-WorkAll, WSSimuAll=0 =============\n");
        pMap->GetRoute(0);
        WorkStatusStaOut = 0;
        WorkStatusSimuAll = 1;
        //WorkStatusGB = 0; WorkStatusSimuAll = 4; // test the GoBorder() quickly
        printf("T1ActSimuAll-WorkAll step0: start to work all. length=%d\n", pMap->GetLength('L'));
    } else if ( WorkStatusSimuAll == 1 ) {
        //int retSO = -1;//StationOut();
        ret = -1;
        if( ret < 0 ) {
            WorkStatusStaOut = -1;
            //WorkStatusSimu = 0;
            //WorkStatusSimuAll = 2;
            WorkStatusSimuAll = 11;
            printf("T1ActSimuAll-WorkAll step1: station out.\n");
        }
    } else if ( WorkStatusSimuAll == 2 ) { // this willnever do
        ret = Work();
        if( ret < 0 ) {
            WorkStatusGH = 0;
            WorkStatusSimuAll = 3;
            printf("T1ActSimuAll-WorkAll step2: work simulation.\n");
        }
    } else if ( WorkStatusSimuAll == 3 ) { // 71->3, 12->3
        ret = GoHome();// go home A* path
        if( ret < 0 ) {
            if ( GoHomeReadPoint() == 1 ) {
                WorkStatusStaIn = 0;
                WorkStatusSimuAll = 5;
            } else {
                WorkStatusGB = 0;
                WorkStatusSimuAll = 4;
            }
            printf("T1ActSimuAll-WorkAll step3: gohome ended.\n");
        }
    } else if ( WorkStatusSimuAll == 4 ) {
        ret = GoBorder();
        if( ret < 0 ) {
            WorkStatusStaIn = 0;
            WorkStatusSimuAll = 5;
            printf("T1ActSimuAll-WorkAll step4: GoBorder ended.\n");
        }
    } else if ( WorkStatusSimuAll == 5 ) { // prepare for the next round of work
        ret = -1;//StationIn();
        if( ret < 0 ) {
            WorkStatusStaIn = -1;
            //WorkStatusSimuAll = -1;
            WorkStatusSimuAll = 0; // loop again and again
            printf("T1ActSimuAll-WorkAll step5: station in ended.\n");
        }
    } else if ( WorkStatusSimuAll == 11 ) { // 1->11, prepare for the RoutePlan lines
        meWorklength = pMap->GetLength('L'); meWorkline = 0; meWorkstep = 0;
        myMower6.fx = 0;  myMower6.fy = 0;
        //flagReverse = 0;
        if ( GoHomeReadPoint() == 1 ) {
            WorkStatusSimuAll = 81;
        } else {
            WorkStatusSimuAll = 12;
        }
        printf("T1ActSimuAll-WorkAll step11: prepare to workall lines. WSSimuAll=%d\n", WorkStatusSimuAll );
    } else if ( WorkStatusSimuAll == 12 ) { // calculate Start or End point to go
        printf("\n");
        if ( meWorkline >= meWorklength ) {
            printf("T1ActSimuAll-WorkAll step12a, scaned all lines: [%d,meWorkline=(%d,%d)], WSSimu=%d\n", meWorklength, meWorkline, meWorkstep, WorkStatusSimu);
            GoHomeClearPoint();
            WorkStatusGH = 0 ;// start GoHome state machine
            WorkStatusSimuAll = 3;
        } else {
            // go to the near
            flagReverse = ((T1Map2 *)pMap)->findAstarNear(flagReverse, myMower6.fx, myMower6.fy, meWorkline);
            if ( flagReverse == 0 ) { //failed
                T1Utils::LogBug("T1ActSimuAll-WorkAll step12b error, findAstarNear() failed.\n");
                WorkStatusSimuAll = -1;
            } else {
                if ( flagReverse > 0 ) { // >=1, from Start to End
                    CutBegin = ((T1Map2 *)pMap)->GetPoint2('L',meWorkline);
                    CutEnd = ((T1Map2 *)pMap)->GetPoint2('l',meWorkline);
                } else {  // <=-1, from End to Start
                    CutBegin = ((T1Map2 *)pMap)->GetPoint2('l',meWorkline);
                    CutEnd = ((T1Map2 *)pMap)->GetPoint2('L',meWorkline);
                }
                printf("T1ActSimuAll-WorkAll step12c, get the RoutePlan2.length=%d\n", pMap->GetLength('2'));
                //int dist = sqrt ( pow(CutBegin.lon-myMower6.fx,2) + pow(CutBegin.lat-myMower6.fy,2)  );
                int dist = abs(flagReverse) * ((T1Map2 *)pMap)->GetLength('g');
                printf("T1ActSimuAll-WorkAll step12d: prepare to workall line:%d, myMower6(%.0f,%.0f),targetPoint(%lld,%lld), dist=%d\n",
                    meWorkline, myMower6.fx, myMower6.fy, CutBegin.lon, CutBegin.lat, dist);
                int cutter = ((T1Map2 *)pMap)->GetLength('c');
                if ( dist < cutter ) { // too close, no need to go
                    WorkStatusSimuAll = 15;
                } else if ( dist < cutter*3 ) { // not far, go without Astar
                    WorkStatusSimuAll = 14;
                } else { // far, go Astar
                    WorkStatusSimuAll = 13;
                }
                WorkStatusSimu = 0;
            }
        }
    } else if ( WorkStatusSimuAll == 13 ) {  // walk follow the Astar path
        ret = Work2(); // SetCutGrid() inside the Work2;
        if( ret < 0 ) {
            WorkStatusSimuAll = 15;
            //meWorkline2 = 0;
            printf("T1ActSimuAll-WorkAll step13: work2().\n");
        }
    } else if ( WorkStatusSimuAll == 14 ) {  // walk directlly to the begin
        //printf("T1ActSimuAll-WorkAll  step14, begin.\n");
        ret = WorkLine(CutBegin);
        if( ret == 0 ) {
            WorkStatusSimuAll = 15;
            printf("T1ActSimuAll-WorkAll  step14a, goto line's begin: rev=%d,(%d,%d,%d), RSW=%d\n", flagReverse, meWorklength, meWorkline, meWorkstep, WorkStatusSimu);
        } else {
            SetCutGrid();
        }    
    } else if ( WorkStatusSimuAll == 15 ) { // walk the RoutePlan' line
        //printf("T1ActSimuAll-WorkAll  step15, begin.\n");
        ret = WorkLine(CutEnd);
        if ( ret>0 ) { // walking
            SetCutGrid();
        } else if ( ret==0 ) { // success goto the target
            SetCutLine(CutBegin,CutEnd);
            printf("T1ActSimuAll-WorkAll  step15a, cut one line to end: rev=%d,(%d,%d,%d), RSW=%d\n", flagReverse, meWorklength, meWorkline, meWorkstep, WorkStatusSimu);
            //printf("TargetS:(%lld,%lld),(%.1f,%.1f)\n", ((T1Map2 *)pMap)->GetPoint2('L',meWorkline-1).lon, ((T1Map2 *)pMap)->GetPoint2('L',meWorkline-1).lat, myMower6.fx, myMower6.fy);
            //printf("TargetE:(%lld,%lld),(%.1f,%.1f)\n", ((T1Map2 *)pMap)->GetPoint2('l',meWorkline-1).lon, ((T1Map2 *)pMap)->GetPoint2('l',meWorkline-1).lat, myMower6.fx, myMower6.fy);
            meWorkline++;
            WorkStatusSimuAll = 16;
        } else if ( ret==-8 ) { // real obstacle detected
            printf("T1ActSimuAll-WorkAll step15->100, obastacle detected. ret=%d\n", ret);
            if ( SetObst(mySgbmWall)==1 ) { // set the real obstacle to gridmap, and re-plan all route lines
                CutEnd.lon = myMower6.fx; CutEnd.lat = myMower6.fy;
                SetCutLine(CutBegin,CutEnd);
                WorkStatusSimuAll = 100;
            } else { // continue;
            }
        } else if ( ret==-5 || ret==-9 ) { // simulated obstacle detected
            printf("T1ActSimuAll-WorkAll step15->100, obastacle detected. ret=%d\n", ret);
            CutEnd.lon = myMower6.fx; CutEnd.lat = myMower6.fy;
            SetCutLine(CutBegin,CutEnd);
            SetObstSimu(ret);
            WorkStatusSimuAll = 100;
        } else {
            printf("T1ActSimuAll-WorkAll step15, unknown WorkLine return(%d)\n", ret); 
        }
    } else if ( WorkStatusSimuAll == 16 ) { // find the nearest uncut RoutePlan's line
        // if all grid have been cut, the return will large then meWorkLength
        meWorkline = ((T1Map2 *)pMap)->findNearestLine((int)myMower6.fx, (int)myMower6.fy);
        printf("T1ActSimuAll-WorkAll step16, find the nearest next line=%d-%d\n", meWorkline, meWorklength);
        WorkStatusSimuAll = 12;
    } else if ( WorkStatusSimuAll == 71 ) { // prepare GoHome()
        GoHomeSavePoint();
        WorkStatusGH = 0;
        WorkStatusSimuAll = 3;
    } else if ( WorkStatusSimuAll == 81 ) { // 11->81, go to the saved point when GoHome();
        printf("T1ActSimuAll-WorkAll step81, read:(%.1f,%.1f,%d,%d)\n", GoHomePoint.fx, GoHomePoint.fy, meWorkline, flagReverse);
        WorkStatusSimuAll = 12; // just goto the last line, not goto the saved point
    } else if ( WorkStatusSimuAll == 100 ) { // re-plan all route lines;
        printf("T1ActSimuAll-WorkAll step100, re-plan the routelines. ret=%d, count=%d\n", ret, count);
        ((T1Map2 *)pMap)->GetRoute3(0);
        count=0;
        WorkStatusSimuAll = 101;
    } else if ( WorkStatusSimuAll == 101 ) { // begin workall with new routePlanLines;
        //pMap->GetRoute(0);
        meWorklength = pMap->GetLength('L'); meWorkline = 0; meWorkstep = 0;
        if ( ++count>5 ) WorkStatusSimuAll = 16;
    } else {
        printf("T1ActSimuAll-WorkAll error status: WSSimuAll=%d, WSSimu=%d\n", WorkStatusSimuAll, WorkStatusSimu);
    }
    return WorkStatusSimuAll;
}

/* set the line to cutted value
 * 2026.03.09: modified by Leo
 */
void T1ActSimuAll::SetCutLine(MapPoint begin, MapPoint end) {
    if ( begin.lon==end.lon && begin.lat==end.lat ) return;
    
    int distx = end.lon-begin.lon;
    int disty = end.lat-begin.lat;
    double dist = sqrt( pow(distx,2) + pow(disty,2) );

    double anglesin = distx / dist;
    double anglecos = disty / dist;
    
    int width =  ((T1Map2 *)pMap)->GetLength('c');
    int dx = anglecos * width / 2;
    int dy = anglesin * width / 2 * -1;
    int step =  ((T1Map2 *)pMap)->GetLength('g');
    int count = dist / step;
    printf("T1ActSimuAll-SetCutLine begin(%lld,%lld) end(%lld,%lld), step=（%d,%d)\n", begin.lon,begin.lat,end.lon,end.lat, step, count);
    float stepx = step * anglesin;
    float stepy = step * anglecos;
    long long x = begin.lon;
    long long y = begin.lat;
    for ( int i=0 ; i<=count; i++ ) {
        Cleft.fx = x - dx;
        Cleft.fy = y - dy;
        Cright.fx = x + dx;
        Cright.fy = y + dy;
        //printf("T1ActSimuAll-SetCutGrid, step 1: xy(%lld,%lld), Cleft(%.1f,%.1f), Cright(%.1f,%.1f)\n", x,y, Cleft.fx, Cleft.fy, Cright.fx, Cright.fy);
        ((T1Map2 *)pMap)->SetGridLineCut((int)Cleft.fx, (int)Cleft.fy, (int)Cright.fx, (int)Cright.fy);    
        //printf("T1ActSimuAll-SetCutLine loop(%d,%d) point(%lld,%lld)\n", count, i, x, y);
        x += stepx;
        y += stepy;
    }
}

/* set the grid to cutted value
 * 20260309: modified by Leo
 */
void T1ActSimuAll::SetCutGrid() {
    //printf("T1ActSimuAll-SetCutGrid, [%d,%d,%d], myMower6=(%.1f,%.1f,%d)\n", meWorkline, meWorkline2, meWorkstep, myMower6.fx, myMower6.fy, myMower6.yaw);

    float width =  ((T1Map2 *)pMap)->GetLength('c') - 0.1;
    float dx = width * cos(3.14159265*myMower6.yaw/180) / 2;
    float dy = width * sin(3.14159265*myMower6.yaw/180) * -1 / 2;
    Cleft.fx = myMower6.fx - dx;
    Cleft.fy = myMower6.fy - dy;
    Cright.fx = myMower6.fx + dx;
    Cright.fy = myMower6.fy + dy;
    //printf("T1ActSimuAll-SetCutGrid, step 1: Cleft(%.1f,%.1f), Cright(%.1f,%.1f)\n", Cleft.fx, Cleft.fy, Cright.fx, Cright.fy);
    ((T1Map2 *)pMap)->SetGridLineCut((int)Cleft.fx, (int)Cleft.fy, (int)Cright.fx, (int)Cright.fy);
    //((T1Map2 *)pMap)->Save2BitMapToFile("../Map/twobitmap.bin");
    //printf("T1ActSimuAll-SetCutGrid, step 2, end\n");
}

/* set the grid to dynamic obstacle
 * 2026.05.20: created by Leo
 */
void T1ActSimuAll::SetObstSimu(int type) {
    PointRobo pfront, pother;
    printf("T1ActSimuAll-SetObstSimu begin, type=%d, myMower6(%.1f,%.1f,%d), pfront(%.1f,%.1f), pother(%.1f,%.1f)\n",
        type, myMower6.fx,myMower6.fy,myMower6.yaw, pfront.fx,pfront.fy, pother.fx,pother.fy);

    float rad = myMower6.yaw * 0.0174532925f;
    float vcos = cos(rad);
    float vsin = sin(rad);

    if ( type==-5 ) { // 50cm, left
        pfront.fx = myMower6.fx + 50.0f * vsin;
        pfront.fy = myMower6.fy + 50.0f * vcos;
        pother.fx = pfront.fx - 10.0f * vcos;
        pother.fy = pfront.fy + 10.0f * vsin;
    } else if ( type==-9 ) { // 90cm, right
        pfront.fx = myMower6.fx + 90.0f * vsin;
        pfront.fy = myMower6.fy + 90.0f * vcos;
        pother.fx = pfront.fx + 30.0f * vcos;
        pother.fy = pfront.fy - 30.0f * vsin;
    } else {
        T1Utils::LogBug("T1ActSimuAll-SetObst error, unknown type.\n");
        return;
    }

    ((T1Map2 *)pMap)->SetGridObstacle(pfront.fx, pfront.fy, pother.fx, pother.fy);

    printf("T1ActSimuAll-SetObstSimu end, myMower6(%.1f,%.1f,%d), pfront(%.1f,%.1f), pother(%.1f,%.1f)\n",
        myMower6.fx,myMower6.fy,myMower6.yaw, pfront.fx,pfront.fy, pother.fx,pother.fy);
}


/* set the grid to wall obstacle based on depth camera data
 * 2026.05.22: created by BY
 * 2026.05.24: modified by Leo, replace myRoboM with myMower6
 * return: 0: failed,    1: success
 */
int T1ActSimuAll::SetObst(Wall wall) {
    PointRobo pleft, pright;
    if ( (wall.leftOffset-6.1)<0.1f && (wall.rightOffset-9.6)<0.1f ) {
        T1Utils::LogBug("T1ActSimuAll-SetObst error, fake wall 33.0,6.1,33.0,9.6 data.\n");
        return 0;
    }
    printf("T1ActSimuAll-SetObst begin, wall(leftDepth=%.1f,leftOffset=%.1f,rightDepth=%.1f,rightOffset=%.1f), myRoboM(%.1f,%.1f,%d)\n",
        wall.leftDepth, wall.leftOffset, wall.rightDepth, wall.rightOffset,
        myMower6.fx, myMower6.fy, myMower6.yaw);

    float rad = myMower6.yaw * 0.0174532925f;
    float vcos = cos(rad);
    float vsin = sin(rad);

    // left edge: depth along robot forward, offset along lateral (camera X axis)
    pleft.fx = myMower6.fx + wall.leftDepth * vsin + wall.leftOffset * vcos;
    pleft.fy = myMower6.fy + wall.leftDepth * vcos - wall.leftOffset * vsin;

    // right edge
    pright.fx = myMower6.fx + wall.rightDepth * vsin + wall.rightOffset * vcos;
    pright.fy = myMower6.fy + wall.rightDepth * vcos - wall.rightOffset * vsin;

    ((T1Map2 *)pMap)->SetGridObstacle(pleft.fx, pleft.fy, pright.fx, pright.fy);    

    printf("T1ActSimuAll-SetObst end, pleft(%.1f,%.1f), pright(%.1f,%.1f)\n",
        pleft.fx, pleft.fy, pright.fx, pright.fy);
    return 1;
}
