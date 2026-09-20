#include "T1ActRoboAll.hpp"
#include "T1Utils.hpp"
//#include <vector>
//#include <string>
#include <cstring>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sys/time.h>
#include <string>
//#include <cstdio>
#include <math.h>
#include <ctime>
#include <dirent.h>

T1ActRoboAll::T1ActRoboAll() {
    printf("T1ActRoboAll-T1ActRoboAll() begin......\n");
    NorthSet(0);
}

T1ActRoboAll::~T1ActRoboAll() {
    
}

/*
 * JudgeFaceNew - Check charging pile alignment via vision module
 * Return: 1=aligned, X value=need X adjust, -1=recognition failed
 * meVisionDistX: Horizontal deviation to charging pile
 * meVisionDistY: Vertical deviation to charging pile
 * meVisionYawAngle: Yaw angle from charging pile
 * 2026.06.26: created by BY
 */
int T1ActRoboAll::JudgeFaceBY() {

    static float lastX = 0, lastY = 0;
    static int lastYaw = 0;
    char tmpstr[200];
    int ret = -1;
    meVisionDistX = myRoboVA.fx;
    meVisionDistY = myRoboVA.fy;
    meVisionYawAngle = myRoboVA.yaw;
    printf("\nT1ActRoboAll-JudgeFaceBY step1, meVisionDistX-Y-Angle=(%f,%f,%d)\n", meVisionDistX, meVisionDistY, meVisionYawAngle);
    
    if(meVisionDistX == -1000 || meVisionDistY == -1000){
        printf("T1ActRoboAll-JudgeFaceBY recognition error, Recognition failed\n");
        meVisionDistX = -1000;
        meVisionDistY = -1000;
        meVisionYawAngle = -1000;
        ret = -1000;
    } else if(meVisionDistX == lastX && meVisionDistY == lastY && meVisionYawAngle == lastYaw){
        ret = -2000;
        printf("T1ActRoboAll-JudgeFaceBY The data is old\n");
    } else if(meVisionDistX <= 6 && meVisionDistX >= -4 && meVisionDistY <= 110 && meVisionDistY >= 90 && abs(meVisionYawAngle) <= 3 ){ 
        printf("T1ActRoboAll-JudgeFaceBY face aligned, meVisionDistX=%f, meVisionDistY=%f\n", meVisionDistX, meVisionDistY);
        ret = 1;   
    }
    lastX = meVisionDistX;
    lastY = meVisionDistY;
    lastYaw = meVisionYawAngle;
    return ret;
}

/*
 * JudgeFaceNew - Check charging pile alignment via vision module
 * Return: 1=aligned, X value=need X adjust, -1=recognition failed
 * meVisionDistX: Horizontal deviation to charging pile
 * meVisionDistY: Vertical deviation to charging pile
 * meVisionYawAngle: Yaw angle from charging pile
 * 2026.03.16: created by BY
 * 2026.04.29: modified by Leo
 */
int T1ActRoboAll::JudgeFaceNew() {
    static float lastX = 0, lastY = 0;
    static int lastYaw = 0;
    char tmpstr[200];
    meVisionDistX = myRoboVA.fx;
    meVisionDistY = myRoboVA.fy;
    meVisionYawAngle = myRoboVA.yaw;
    printf("\nT1ActRoboAll-JudgeFaceNew step1, meVisionDistX-Y-Angle=(%f,%f,%d)\n", meVisionDistX, meVisionDistY, meVisionYawAngle);
    if ((meVisionDistX == -1000 || meVisionDistY == -1000)
        || (meVisionDistX == lastX && meVisionDistY == lastY && meVisionYawAngle == lastYaw)) {
        return -1000;
    }
    lastX = meVisionDistX;
    lastY = meVisionDistY;
    lastYaw = meVisionYawAngle;
    return 1;
}

int T1ActRoboAll::JudgeCharge() {
    if (FlagInCharge == 1) {
        // Already detected in charging station by ProcessBatteryVoltage()
        RoboAct('g', 0);  // Stop the robot
        printf("T1ActRoboAll-JudgeCharge: Arrived at charging station, stopped and cleared flag\n");
        return 1;  // Successfully arrived at charging station
    }

    return 0;
}

/*
 * ProcessVisionResult - Process vision detection result and determine next state
 * @param res: Result from JudgeFaceNew()
 *   - 1: aligned
 *   - -1000: not detected
 *   - other: need adjust
 * @param isEntered: Whether robot has entered the recognition range
 * @return: Next WorkStatusStaIn value
 * 2026.05.13: optimized by Roo based on actual running data
 */
/*
int T1ActRoboAll::ProcessVisionResult(int res) {
    // Priority 1: Not detected
    if (res == -1000) {
        printf("T1ActRoboAll-ProcessVisionResult: Not detected, search for pile\n");
        return 3;  // State 3: Search
    } else if (meVisionDistY > 110 || meVisionDistY < 80) {
        printf("T1ActRoboAll-ProcessVisionResult: Y distance out of range: %.1f\n", meVisionDistY);
        return 9;  // State 9: Adjust Y
    } else if (meVisionDistX > 10 || meVisionDistX < -10) {
        printf("T1ActRoboAll-ProcessVisionResult: X distance out of range: %.1f\n", meVisionDistX);
        return 10;  // State 10: Adjust X
    } else if (abs(meVisionYawAngle) > 2) {
        printf("T1ActRoboAll-ProcessVisionResult: Yaw angle too large: %d\n", meVisionYawAngle);
        return 19;  // State 19: Adjust yaw
    } else  {
        printf("T1ActRoboAll-ProcessVisionResult: Aligned successfully\n");
        return 13;  // State 13: Aligned
    }
    // Default: Fine adjustment needed
    printf("T1ActRoboAll-ProcessVisionResult: Fine adjustment needed\n");
}
*/
/* Set charging station direction as "North" (0 degrees)
 */
void T1ActRoboAll::SetStationNorth() {
    // Calculate offset to make current IMU direction = 0 (North)
    myDiffMI.yaw = -myRoboI.yaw;
    
    // Normalize angle to [0, 360) range
    if (myDiffMI.yaw < 0) myDiffMI.yaw += 360;
    if (myDiffMI.yaw >= 360) myDiffMI.yaw -= 360;
    
    // Update robot's corrected yaw (should be 0 = North)
    myRoboM.yaw = myRoboI.yaw + myDiffMI.yaw;
    if (myRoboM.yaw >= 360) myRoboM.yaw -= 360;
    if (myRoboM.yaw < 0) myRoboM.yaw += 360;
    
    printf("T1ActRoboAll-SetStationNorth: Charging station North calibration\n");
    printf("  IMU yaw: %d degrees\n", myRoboI.yaw);
    printf("  North offset (myDiffMI.yaw): %d degrees\n", myDiffMI.yaw);
    printf("  Robot corrected yaw (should be 0): %d degrees\n", myRoboM.yaw);
}

/* do some actions when MapByTrail begin
 * return==1: working,  return<0: finished work,
 * 2026.03.15: creat by Leo;
 */
int T1ActRoboAll::WorkTrail() {
    //printf("T1ActRobo::WorkTrail begin, WST=%d.\n", WorkStatusT);
    int ret=StationOut();
    if ( ret<0 ) {
        WorkStatusT=1; // begin to record trail
        printf("T1ActRobo-WorkTrail end, station out.\n");
    }
    return WorkStatusT;
}

/* Out the charge Station
 * Status: -1=Completed, 0=Initialized, 1=Forward exit, 2= Triangle northfinding, 3= Northfinding successful, 4= Northfinding failed
 * Northbound determination: ployaw==1000 indicates that the northbound direction has been determined
 * 2026.05.04: created by Xs
 * 2026.05.05: midified by Leo
 * 2026.05.22: midified by Xs
 */
int T1ActRoboAll::StationOut() {
    int ret;
    int debug=4; // Set to 1 to enable debug prints, 0 to disable
    static int count = 0;
    int countmax = 6;
    int step = meFact.StationLength/countmax;
 
    if ( WorkStatusStaOut<0 || FlagPause==1 ) {
        printf("T1ActRoboAll-StationOut: paused\n");
    } else if ( WorkStatusStaOut==0) { // prepare to walk out the charge station
        //printf("T1ActRoboAll-StationOut[0]: ready to walk out\n");
        RoboAct('g',0);
        FlagStop=0;
        WorkStatusStaOut = 1;
        FlagSetSpeed = 1;
        posFixedVA = 0;
    } else if ( WorkStatusStaOut == 1) { // judge if in charge station
        ret = 1;
        if ( ret==1 ) {
            meAngleStation = myRoboI.yaw;
            if ( meStageNorth==0 ) { // read saved north angle
                if ( AngleStationNorthRead()==1 ) {
                    if (debug>2) printf("T1ActRoboAll-StationOut step1a, read North angle:(%d,%d)\n", meAngleNorth, meDiffNorth);
                    meStageNorth=1;
                }
            }
            if ( meFact.DriveWheel==0 ) { // frontwheel drive
                meDiffStation =  -1*meAngleStation;
                RoboAct('w',meFact.StationLength); // RTK
                if (debug>2) printf("T1ActRoboAll-StationOut step1b w: In charge station, meAngleStation=%d, DiffStation=%d\n", meAngleStation, meDiffStation);
                WorkStatusStaOut = 2;
            } else { // visual mower, rearwheel drive
                meDiffStation =  -1*meAngleStation + 180;
                count = 0;
                if (debug>2) printf("T1ActRoboAll-StationOut step1b S: In charge station, meAngleStation=%d, DiffStation=%d\n", meAngleStation, meDiffStation);
                WorkStatusStaOut = 101;
            }
            myRoboM.fx=0; myRoboM.fy=meFact.StationLength*(-1); myRoboM.yaw=myRoboI.yaw+meDiffStation;
            if (debug>2) printf("T1ActRoboAll-StationOut step1c: In charge. meAngleStation=%d, DiffStation=%d, myRoboI=%d, myRoboM=%d,%.1f,%.1f\n", meAngleStation, meDiffStation, myRoboI.yaw, myRoboM.yaw, myRoboM.fx, myRoboM.fy);
        } else {
            if (debug>2) printf("T1ActRoboAll-StationOut step1d: Not in charge.\n");
            WorkStatusStaOut = 2;
        }
    } else if ( WorkStatusStaOut == 2 ) { // walkout the charge station
        ret = RoboAct('=', 0);
        if ( ret==0 ) { // walking
            ;
        } else if ( ret<0 ) { // walk enough time
            if (debug>2) printf("T1ActRoboAll-StationOut step2b: walk out success。 ret=%d, mRoboM=(%.1f,%.1f)\n", ret, myRoboM.fx, myRoboM.fy);
            count =0;
            WorkStatusStaOut = 3;
        } else { // ret > 0
            if (debug>2) printf("T1ActRoboAll-StationOut step2c: some error when walk out the station。 ret=%d, mRoboM=(%.1f,%.1f,%d)\n", ret, myRoboM.fx, myRoboM.fy, myRoboM.yaw);
            count =0;
            WorkStatusStaOut = 3;
        }
    } else if (WorkStatusStaOut == 3) { // end, prepare to workAll
        if ( meFact.DriveWheel!=0 ) {
            count++;
            if ( count>=200 ) {
                if (debug>2) printf("T1ActRoboAll-StationOut step3->4, count=%d, myRoboM=(%.0f,%.0f,%d)\n", count, myRoboM.fx,myRoboM.fy,myRoboM.yaw);
                WorkStatusStaOut = 4;
            }
            if ( posFixedVA == 4 ) {
                if(Tag2Robot()) {
                    PosiSetByVisA();
                    posFixedVA = 0;
                    if (debug>3) printf("T1ActRoboAll-StationOut step3, position set by VA, count=%d, myRoboM=(%.1f,%.1f,%d), myDiffMI.yaw=%d, myRoboI.yaw=%d, step=%d, ret=%d\n",
                        count, myRoboM.fx, myRoboM.fy, myRoboM.yaw, myDiffMI.yaw, myRoboI.yaw,step,ret);
                }
            }
        } else {
            WorkStatusStaOut = 4;
        }
    } else if (WorkStatusStaOut == 4) { // end, prepare to workAll
        if ( meFact.DriveWheel==0 ) {
            myRoboM.fx=0; myRoboM.fy=0;
            if (debug>2) printf("T1ActRoboAll-StationOut step4, RTK robot: set position to origin (0,0)\n");
        }
        if (debug>2) printf("T1ActRoboAll-StationOut step4, mapBase(%lld,%lld),  meStageNorth=%d\n", mapBase.lon, mapBase.lat, meStageNorth);
        if ( mapBase.lon==-1 && mapBase.lat==-1 ) { // demo map
            myDiffMI.yaw=meDiffStation;
        } else { // not demo map
            if ( meStageNorth>0 ) { // the north is got after StationOut
                myDiffMI.yaw=meDiffNorth;
            } else {
                myDiffMI.yaw=meDiffStation;
            }
        }
        myRoboM.yaw = myRoboI.yaw + myDiffMI.yaw;
        //for ( int i=0; i<360; i+=10 ) {// for test
        //    myRoboM.fx=0; myRoboM.fy=0; myRoboM.yaw = i;// for test
        //    printf("\n>>>>>>>test: i=%d\n",i);// for test
            /*
            if ( posFixedVA == 4 ) {
                if (debug>3) printf("T1ActRoboAll-StationOut step3a, before set by VA, myRoboM=(%.1f,%.1f,%d), myDiffMI.yaw=%d, myRoboI.yaw=%d\n",
                    myRoboM.fx, myRoboM.fy, myRoboM.yaw, myDiffMI.yaw, myRoboI.yaw);
                PosiSetByVisA();
                posFixedVA = 0;
            }
            */
        //}// for test
        if (debug>2) printf("T1ActRoboAll-StationOut step4a, set meRoboM.yaw=%d, myDiffMI.yaw=%d, myRoboI.yaw=%d\n", myRoboM.yaw, myDiffMI.yaw, myRoboI.yaw);
        if (debug>0) printf("T1ActRoboAll-StationOut end.\n--------\n");
        strcpy(meStrSendVis,"4"); // range: 0--9, default is 9, 0 is always high resolution
        WorkStatusStaOut = 1;
        WorkStatusStaOut = -1;
    } else if ( WorkStatusStaOut == 101 ) { // this is for Visual type mower with rearwheel drive
        RoboAct('s',step);
        WorkStatusStaOut = 102;
    } else if ( WorkStatusStaOut == 102 ) { // this is for Visual type mower with rearwheel drive
        ret = RoboAct('=', 0);
        if ( ret==0 ) {
            ;
        } else if ( ret<0 ) {
        } else { // ret>0
            if (debug>5) printf("T1ActRoboAll-StationOut step102a, count=%d/18, ret=%d, myRoboM=(%.0f,%.0f,%d)\n", count, ret, myRoboM.fx,myRoboM.fy,myRoboM.yaw);

            if (debug>2) printf("T1ActRoboAll-StationOut step102c, loop=(%d/%d), step=%d, ret=%d\n", count, countmax, step,ret);
            RoboPosChange('s',step,ret);

            if ( ++count<countmax*2 ) {
                WorkStatusStaOut = 101;
            } else {
                if (debug>2) printf("T1ActRoboAll-StationOut step 102->103\n");
                WorkStatusStaOut = 103;
            }
        }
    } else if ( WorkStatusStaOut == 103 ) { // this is for Visual type mower with rearwheel drive
        RoboAct('g', 0);
        count = 0;
        WorkStatusStaOut = 110;
    } else if ( WorkStatusStaOut == 110 ) { // delay some time, for the mower stop totally
        if ( ++count>12 ) {
            FlagStop=0;
            count = 0;
            WorkStatusStaOut = 111;
            if (debug>2) printf("T1ActRoboAll-StationOut step 110->111, count=%d\n", count);
        }
    } else if (WorkStatusStaOut == 111) {
        RoboAct('W',step);
        WorkStatusStaOut = 112;
    } else if ( WorkStatusStaOut == 112 ) { // this is for Visual type mower with rearwheel drive
        ret = RoboAct('=', 0);
        if ( ret==0 ) {
            ;
        } else if ( ret<0 ) {
        } else { // ret>0
            if (debug>5) printf("T1ActRoboAll-StationOut step112a, count=%d/10, ret=%d, myRoboM=(%.0f,%.0f,%d)\n", count, ret, myRoboM.fx,myRoboM.fy,myRoboM.yaw);

            /*if ( isAprilTagDetected() ) {
                if (debug>3) printf("T1ActRoboAll-StationOut step112b, meVisionDistX-Y-YAW=(%.1f,%.1f,%d)\n",
                    meVisionDistX, meVisionDistY, meVisionYawAngle);
                posFixedVA = 4;
            }*/

            if (debug>2) printf("T1ActRoboAll-StationOut step112c, loop=(%d/%d), step=%d\n", count, countmax, step);
            RoboPosChange('W',step,ret);

            if ( ++count<(countmax) ) {
                WorkStatusStaOut = 111;
            } else {
                if (debug>2) printf("T1ActRoboAll-StationOut step 112->113\n");
                WorkStatusStaOut = 113;
            }
        }
    } else if ( WorkStatusStaOut == 113 ) { // this is for Visual type mower with rearwheel drive
        RoboAct('g', 0);
        count = 0;
        FlagSetSpeed = 0 ;
        WorkStatusStaOut = 3;
    } else {
        printf("T1ActRoboAll-StationOut: ERROR status=%d\n", WorkStatusStaOut);
    }
    return WorkStatusStaOut;
}

/*
 * Out the charge Station
 * Status: -1=Completed, 0=Initialized, 1=Forward exit, 2= Triangle northfinding, 3= Northfinding successful, 4= Northfinding failed
 * Northbound determination: ployaw==1000 indicates that the northbound direction has been determined
 * 2026.03.06: created by Leo
 */
/*
int T1ActRoboAll::StationOut() {
    int ret;
    int debug=1; // Set to 1 to enable debug prints, 0 to disable
    if ( WorkStatusStaOut<0 || FlagPause==1 ) {
        printf("T1ActRoboAll-StationOut: paused\n");
    } else if ( WorkStatusStaOut==0) { // prepare to walk out the charge station
        //printf("T1ActRoboAll-StationOut[0]: ready to walk out\n");
        //flagOnCharger = -1;  // Reset flag for new detection
        RoboAct('g',0);
        strcpy(meStrSendVis,"0"); // range: 0--9, default is 9, 0 is always high resolution         WorkStatusStaOut = 1;
    } else if ( WorkStatusStaOut == 1) { // judge if in charge station
        ret = 1;//DetectChargeStation();
        if ( ret==1 ) {
            meAngleStation = myRoboI.yaw;
            if ( meStageNorth==0 ) { // read saved north angle
                if ( AngleStationNorthRead()==1 ) {
                    if (debug>2) printf("T1ActRoboAll-StationOut step1a, read North angle:(%d,%d)\n", meAngleNorth, meDiffNorth);
                    meStageNorth=1;
                }
            }
            if ( meFact.DriveWheel==0 ) { // frontwheel drive
                meDiffStation =  -1*meAngleStation;
                RoboAct('w',meFact.StationLength); // RTK
                if (debug>2) printf("T1ActRoboAll-StationOut step1b w: In charge station, meAngleStation=%d, DiffStation=%d\n", meAngleStation, meDiffStation);
            } else {
                meDiffStation =  -1*meAngleStation + 180;
                RoboAct('S',meFact.StationLength); // Vision
                if (debug>2) printf("T1ActRoboAll-StationOut step1b S: In charge station, meAngleStation=%d, DiffStation=%d\n", meAngleStation, meDiffStation);
            }
            myRoboM.fx=0; myRoboM.fy=meFact.StationLength*(-1); myRoboM.yaw=myRoboI.yaw+meDiffStation;
            //RoboAct('w',120);
            if (debug>2) printf("T1ActRoboAll-StationOut step1c: In charge. meAngleStation=%d, DiffStation=%d, myRoboI=%d\n", meAngleStation, meDiffStation, myRoboI.yaw);
        } else {
            if (debug>2) printf("T1ActRoboAll-StationOut step1d: Not in charge.\n");
        }
        WorkStatusStaOut = 2;
    } else if ( WorkStatusStaOut == 2 ) { // walkout the charge station
        ret = RoboAct('=', 0);
        if ( ret == 0 ) { // walking
            ;
        } else if ( ret < 0 ) { // walk enough time
            if (debug>2) printf("T1ActRoboAll-StationOut step2b: walk out success。 ret=%d, mRoboM=(%.1f,%.1f)\n", ret, myRoboM.fx, myRoboM.fy);
            WorkStatusStaOut = 3;
        } else { // ret > 0
            if (debug>2) printf("T1ActRoboAll-StationOut step2c: some error when walk out the station。 ret=%d, mRoboM=(%.1f,%.1f,%d)\n", ret, myRoboM.fx, myRoboM.fy, myRoboM.yaw);
            WorkStatusStaOut = 3;
        }
    } else if (WorkStatusStaOut == 3) { // end, prepare to workAll
        myRoboM.fx=0; myRoboM.fy=0;
        if (debug>2) printf("T1ActRoboAll-StationOut step3, mapBase(%lld,%lld),  meStageNorth=%d\n", mapBase.lon, mapBase.lat, meStageNorth);
        if ( mapBase.lon==-1 && mapBase.lat==-1 ) { // demo map
            myDiffMI.yaw=meDiffStation;
        } else { // not demo map
            if ( meStageNorth>0 ) { // the north is got after StationOut
                myDiffMI.yaw=meDiffNorth;
            } else {
                myDiffMI.yaw=meDiffStation;
            }
        }
        myRoboM.yaw = myRoboI.yaw + myDiffMI.yaw;
        //for ( int i=0; i<360; i+=10 ) {// for test
        //    myRoboM.fx=0; myRoboM.fy=0; myRoboM.yaw = i;// for test
        //    printf("\n>>>>>>>test: i=%d\n",i);// for test

        // if( isAprilTagDetected() ) {
        //     posFixedVA = 4;
        //     PosiSetByVisA();
        //     posFixedVA = 0;
        // }

        //}// for test
        if (debug>2) printf("T1ActRoboAll-StationOut step3a, set meRoboM.yaw=%d, myDiffMI.yaw=%d, myRoboI.yaw=%d\n", myRoboM.yaw, myDiffMI.yaw, myRoboI.yaw);
        if (debug>0) printf("T1ActRoboAll-StationOut end, step3.\n");
        WorkStatusStaOut = -1;
    } else {
        printf("T1ActRoboAll-StationOut: ERROR status=%d\n", WorkStatusStaOut);
    }
    return WorkStatusStaOut;
}
*/

/*
 * StationInRTK - Robot auto return to charging pile
 * Return: Current process status (-1=Finish/Error, others=in progress)
 * 2026.06.26: created by BY
 */
int T1ActRoboAll::StationInRTK() {
    int ret;
    int res = 0;
    static int count = 0;

    if (WorkStatusStaIn < 0 || FlagPause == 1) { // Stopped or paused
        return WorkStatusStaIn;
    } else if (WorkStatusStaIn == 0) { // Initialize return to charge station
        strcpy(meStrSendVis,"0");
        WorkStatusStaIn = 1;
        std::cout << "T1ActRoboAll-StationInRTK Init Status0->1: Back to origin point (0,0)." << std::endl;
    } else if (WorkStatusStaIn == 1) { // return to (0,0)
        WorkStatusStaIn = 2;
        IsCharing = 0;
        std::cout << "T1ActRoboAll-StationInRTK Step1 Status1->2: Turn body to face charge station." << std::endl;
    } else if (WorkStatusStaIn == 2) { // turn to face charge station, prepare for photo recognition
        ret = RoboAct('=', 0);
        if (ret < 0) {
            int diff = ((meAngleStation + myDiffMI.yaw + 180 - myRoboM.yaw + 540) % 360) - 180;
            if (diff > 0) {
                RoboAct(meCmdC, diff);
            } else {
                RoboAct(meCmdZ, abs(diff));
            }
        } else if (ret == 0) {
            ;
        } else {
            WorkStatusStaIn = 3;
            std::cout << "T1ActRoboAll-StationInRTK Step2 Status2->3: Turn to face charge station success." << std::endl;
        }
    } else if (WorkStatusStaIn == 3) {
        count++;
        if (count % 75 == 0) {
            res = JudgeFaceBY();
            if (res == 1) {
                WorkStatusStaIn = 100;
                std::cout << "T1ActRoboAll-StationInRTK Step3 Status3->100: Vision position check pass, enter precise docking stage." << std::endl;
            } else {
                if(res==-2000){
                    std::cout << "T1ActRoboAll-StationInRTK Step3 the data is old." << std::endl;
                } else if (meVisionDistX == -1000 && meVisionDistY == -1000) {
                    WorkStatusStaIn = 9;
                    std::cout << "T1ActRoboAll-StationInRTK Step3 Status3->9: Vision lose station, start search logic." << std::endl;
                } else if (abs(meVisionYawAngle) > 3) {
                    WorkStatusStaIn = 4;
                    std::cout << "T1ActRoboAll-StationInRTK Step3 Status3->4: Yaw angle offset over threshold, enter angle adjust." << std::endl;
                } else if ((meVisionDistX > 20 || meVisionDistX < -18)) {
                    WorkStatusStaIn = 5;
                    std::cout << "T1ActRoboAll-StationInRTK Step3 Status3->5: X axis offset abnormal, prepare turn back." << std::endl;
                } else if (meVisionDistY < 90 && meVisionDistY > 0) {
                    WorkStatusStaIn = 7;
                    std::cout << "T1ActRoboAll-StationInRTK Step3 Status3->7: Y distance too close, forward adjust." << std::endl;
                } else if (meVisionDistY > 110) {
                    WorkStatusStaIn = 8;
                    std::cout << "T1ActRoboAll-StationInRTK Step3 Status3->8: Y distance too far, backward adjust." << std::endl;
                } else if (meVisionDistX > 6 || meVisionDistX < -4) {
                    WorkStatusStaIn = 5;
                    std::cout << "T1ActRoboAll-StationInRTK Step3 Status3->5: X offset out of standard range, turn adjust." << std::endl;
                } else {
                    std::cout << "T1ActRoboAll-StationInRTK Step3 Status3: Station position within tolerance, hold current state." << std::endl;
                }
            }
        }
    } else if (WorkStatusStaIn == 4) { // Judge whether it is necessary to adjust the angle
        ret = RoboAct('=', 0);
        if (ret < 0) {
            if (meVisionYawAngle > 0) {
                RoboAct(meCmdZ, abs(meVisionYawAngle));
            } else {
                RoboAct(meCmdC, abs(meVisionYawAngle));
            }
        } else if (ret == 0) {
            ;
        } else {
            meAngleStation -= meVisionYawAngle;
            std::cout << "T1ActRoboAll-StationInRTK Step4 Status4->3: Adjust vision yaw angle finished, offset=" << meVisionYawAngle << "." << std::endl;
            WorkStatusStaIn = 3;
        }
    } else if (WorkStatusStaIn == 5) { // Turn around, prepare to back into charge station
        ret = RoboAct('=', 0);
        if (ret < 0) {
            int diff;
            if (meVisionDistX < 0) {
                diff = -90;
            } else {
                diff = 90;
            }
            if (diff > 0) {
                RoboAct(meCmdC, diff);
            } else if (diff < 0) {
                RoboAct(meCmdZ, abs(diff));
            }
        } else if (ret == 0) {
            ;
        } else {
            WorkStatusStaIn = 6;
            std::cout << "T1ActRoboAll-StationInRTK StepX Status5->6: X axis turn action complete, enter backward align." << std::endl;
        }
    } else if (WorkStatusStaIn == 6) { // Backward into charge station
        ret = RoboAct('=', 0);
        if (ret < 0) {
            RoboAct('W', abs(meVisionDistX) + 12);
            std::cout << "T1ActRoboAll-StationInRTK Step6 Status6: Backward adjust for station align." << std::endl;
        } else if (ret == 0) {
            ;
        } else {
            WorkStatusStaIn = 2;
            std::cout << "T1ActRoboAll-StationInRTK Step6 Status6->2: X backward adjust finish, return yaw align step." << std::endl;
        }
    } else if (WorkStatusStaIn == 7) {
        int distance = 110 - meVisionDistY;
        ret = RoboAct('=', 0);
        if (ret < 0) {
            RoboAct('S', distance);
            std::cout << "T1ActRoboAll-StationInRTK Step7 Status7: Too close to station, move forward adjust distance." << std::endl;
        } else if (ret == 0) {
            ;
        } else {
            WorkStatusStaIn = 2;
            std::cout << "T1ActRoboAll-StationInRTK Step7 Status7->2: Close distance adjust complete, return yaw align step." << std::endl;
        }
    } else if (WorkStatusStaIn == 8) {
        int distance = meVisionDistY - 90;
        ret = RoboAct('=', 0);
        if (ret < 0) {
            RoboAct('W', distance);
            std::cout << "T1ActRoboAll-StationInRTK Step8 Status8: Too far from station, move backward adjust distance." << std::endl;
        } else if (ret == 0) {
            ;
        } else {
            WorkStatusStaIn = 2;
            std::cout << "T1ActRoboAll-StationInRTK Step8 Status8->2: Far distance adjust complete, return yaw align step." << std::endl;
        }
    } else if (WorkStatusStaIn == 9) {
        ret = RoboAct('=', 0);
        if (ret < 0) {
            RoboAct(meCmdC, 90);
        } else if (ret == 0) {
            ;
        } else {
            WorkStatusStaIn = 10;
            std::cout << "T1ActRoboAll-StationInRTK Step9 Status9->10: Station lost, turn right to search." << std::endl;
        }
    } else if (WorkStatusStaIn == 10) {
        ret = RoboAct('=', 0);
        if (ret < 0) {
            RoboAct('W', 60);
        } else if (ret == 0) {
            ;
        } else {
            WorkStatusStaIn = 11;
            std::cout << "T1ActRoboAll-StationInRTK Step10 Status10->11: Station lost, move forward 40cm to search." << std::endl;
        }
    } else if (WorkStatusStaIn == 11) {
        ret = RoboAct('=', 0);
        if (ret < 0) {
            RoboAct(meCmdZ, 90);
        } else if (ret == 0) {
            ;
        } else {
            count++;
            if (count % 75 == 0) {
                if (JudgeFaceBY() == -1000) {
                    WorkStatusStaIn = 12;
                    std::cout << "T1ActRoboAll-StationInRTK Step11 Status11->12: Still no station detected, continue position adjust." << std::endl;
                } else {
                    WorkStatusStaIn = 2;
                    std::cout << "T1ActRoboAll-StationInRTK Step11 Status11->2: Station re-detected, back to align step." << std::endl;
                }
            }
        }
    } else if (WorkStatusStaIn == 12) {
        ret = RoboAct('=', 0);
        if (ret < 0) {
            RoboAct(meCmdZ, 90);
        } else if (ret == 0) {
            ;
        } else {
            WorkStatusStaIn = 13;
            std::cout << "T1ActRoboAll-StationInRTK Step12 Status12->13: Station lost, turn left to search." << std::endl;
        }
    } else if (WorkStatusStaIn == 13) {
        ret = RoboAct('=', 0);
        if (ret < 0) {
            RoboAct('W', 120);
        } else if (ret == 0) {
            ;
        } else {
            WorkStatusStaIn = 14;
            std::cout << "T1ActRoboAll-StationInRTK Step13 Status13->14: Station lost, long forward move to search." << std::endl;
        }
    } else if (WorkStatusStaIn == 14) {
        ret = RoboAct('=', 0);
        if (ret < 0) {
            RoboAct(meCmdZ, 90);
        } else if (ret == 0) {
            ;
        } else {
            count++;
            if (count % 75) {
                if (JudgeFaceBY() == -1000) {
                    WorkStatusStaIn = -1;
                    std::cout << "T1ActRoboAll-StationInRTK Step14 Status14->-1: Max search times, no station found, exit return charge." << std::endl;
                } else {
                    WorkStatusStaIn = 2;
                    std::cout << "T1ActRoboAll-StationInRTK Step14 Status14->2: Station re-detected, back to align step." << std::endl;
                }
            }
        }
    } else if (WorkStatusStaIn == 100) {
        ret = RoboAct('=', 0);
        if (ret < 0) {
            if (meVisionDistX >= -4 && meVisionDistX <= -2) {
                RoboAct('Z', 180);
                std::cout << "T1ActRoboAll-StationInRTK Step100 Status100: Turn left 180 degree to back toward station." << std::endl;
            } else if (meVisionDistX >= -2 && meVisionDistX <= 1) {
                RoboAct('Z', 176);
                std::cout << "T1ActRoboAll-StationInRTK Step100 Status100: Slight left turn 176 degree." << std::endl;
            } else {
                RoboAct('C', 176);
                std::cout << "T1ActRoboAll-StationInRTK Step100 Status100: Turn right 176 degree to back toward station." << std::endl;
            }
        } else if (ret == 0) {
            ;
        } else {
            WorkStatusStaIn = 101;
            std::cout << "T1ActRoboAll-StationInRTK Step100 Status100->101: Turn back action complete, enter pre-distance adjust." << std::endl;
        }
    } else if (WorkStatusStaIn == 101) {
        ret = RoboAct('=', 0);
        std::cout << "T1ActRoboAll-StationInRTK Step101 Status101: Move action ret=" << ret << "." << std::endl;
        if (ret < 0) {
            if (abs(meVisionDistY) > 70) {
                RoboAct('S', abs(meVisionDistY) - 30);
                std::cout << "T1ActRoboAll-StationInRTK Step101 Status101: Backward adjust to 55cm pre-stop position." << std::endl;
            } else {
                WorkStatusStaIn = 102;
                std::cout << "T1ActRoboAll-StationInRTK Step101 Status101->102: Pre-distance adjust finish, enter slow fine docking." << std::endl;
            }
        } else if (ret == 0) {
            ;
        } else {
            WorkStatusStaIn = 102;
            std::cout << "T1ActRoboAll-StationInRTK Step101 Status101->102: Pre-distance action done, enter slow fine docking." << std::endl;
        }
    } else if (WorkStatusStaIn == 102) {
        ret = RoboAct('=', 0);
        if (ret < 0) {
            RoboAct('s', 80);
            std::cout << "T1ActRoboAll-StationInRTK Step102 Status102: Slow backward fine tune 70cm to dock station." << std::endl;
        } else if (ret == 0) {
            if (JudgeCharge() == 1) {
                RoboAct('g', 0);
                WorkStatusStaIn = -1;
                std::cout << "T1ActRoboAll-StationInRTK Step102 Status102->-1: Charging signal detected, stop all movement, docking finish." << std::endl;
            }
        } else {
            WorkStatusStaIn = 103;
            std::cout << "T1ActRoboAll-StationInRTK Step102 Status102->103: Slow backward fine tune complete, charge detection double check." << std::endl;
        }
    } else if (WorkStatusStaIn == 103) {
        if (JudgeCharge() == 1) {
            WorkStatusStaIn = -1;
            std::cout << "T1ActRoboAll-StationInRTK Step103 Status103->-1: Docking complete, robot is charging now." << std::endl;
        } else {
            WorkStatusStaIn = 104;
            std::cout << "T1ActRoboAll-StationInRTK Step103 Status103->104: No charging signal detected, retry backward align." << std::endl;
        }
    } else if (WorkStatusStaIn == 104) {
        ret = RoboAct('=', 0);
        if (ret < 0) {
            RoboAct('W', 100);
            std::cout << "T1ActRoboAll-StationInRTK Step104 Status104: Docking failed, retry backward align station." << std::endl;
        } else if (ret == 0) {
            ;
        } else {
            WorkStatusStaIn = 2;
            std::cout << "T1ActRoboAll-StationInRTK Step104 Status104->2: Retry backward align finish, back to vision yaw adjust." << std::endl;
        }
    }

    return WorkStatusStaIn;
}

/*
 * StationInVision - Robot auto return to charging pile
 * Return: Current process status (-1=Finish/Error, others=in progress)
 * 2026.04.11: created by BY
 * 2026.05.25: Modified by Xs
 * 2026.06.08: Modified by Xs
 */
int T1ActRoboAll::StationInVision() {
    int ret;
    if ( WorkStatusStaIn < 0 || FlagPause == 1) { // Stopped or paused
        return WorkStatusStaIn;
    } else if ( WorkStatusStaIn == 0 ) {// Initial state
        if( IsCharing == 1 ){
            printf("T1ActRoboAll-StationInVision step0, Initial state\n");
            pmyVision->SetAprilTagSize(GetAprilTagSize(1));
            WorkStatusStaIn = 1;
            IsCharing = 0;
        }else{
            printf("T1ActRoboAll-StationInVision step0, don't need Initial state\n");
            pmyVision->SetAprilTagSize(GetAprilTagSize(1));
            WorkStatusStaIn = 1;
        }     
    } else if ( WorkStatusStaIn == 1) { // turn to face charge station
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            printf("T1ActRoboAll-StationInVision step1, turn to face charge station\n");
            int diff = ((meAngleStation + myDiffMI.yaw - myRoboM.yaw + 540) % 360) - 180;
            if ( diff > 0 ){
                //RoboAct('C', diff);
                RoboAct(meCmdC, diff);
            } else {
                //RoboAct('Z', abs(diff));
                RoboAct(meCmdZ, abs(diff));
            }
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            WorkStatusStaIn = 2;
            printf("T1ActRoboAll-StationInVision step1, turn to face charge station success.\n");
        }
    } else if ( WorkStatusStaIn == 2 ) { // detect the april tag
        int res = JudgeFaceNew();
        if ( meVisionDistY <= 100 && meVisionDistY >= 80 && meVisionDistX > -35 && meVisionDistX < 35 ) { //
            mevisEnterCnt = 1;
        }
        if ( res == -1000 ) { // didn't identified the charging pile 
            printf("T1ActRoboAll-StationInVision step2, didn't identified the charging pile.\n");
            WorkStatusStaIn = 3;
        } else if ( abs(meVisionYawAngle) > 4 ) {
            WorkStatusStaIn = 19;
        } else if ( (meVisionDistY > 100 || meVisionDistY < 80) && mevisEnterCnt == 0 ) { // check y position
            printf("T1ActRoboAll-StationInVision step2, too far or too close. prepare to adjust y position\n");
            WorkStatusStaIn = 9;
        } else if ( (meVisionDistX > 35 || meVisionDistX < -35) && mevisEnterCnt == 0 ) { // check x position
            printf("T1ActRoboAll-StationInVision step2, too far or too close. prepare to adjust x position\n");
            WorkStatusStaIn = 10;
        } else if ( res == 1 && mevisEnterCnt == 1 ) { // Able to start pile driving
            printf("T1ActRoboAll-StationInVision step2, Able to start pile driving\n");
            WorkStatusStaIn = 13; 
        } else if ( res != 1 || mevisEnterCnt == 1 ) { // Not within the appropriate threshold
            printf("T1ActRoboAll-StationInVision step2, Not within the appropriate threshold need adjust\n");
            WorkStatusStaIn = 12; 
        } 
    } else if (WorkStatusStaIn == 3) { // turn right 90
        ret = RoboAct('=',0);
        if (ret < 0) {
            //RoboAct('C', 90);
            RoboAct(meCmdC, 90);
        }else if ( ret == 0 ) { // Action......
            ;
        }else{
            printf("T1ActRoboAll-StationInNew step3, recognition failed, try turning right to adjust the position.\n");
            WorkStatusStaIn = 4;
        }
    } else if(WorkStatusStaIn == 4){ // walkforward 60cm
        ret = RoboAct('=',0);
        if (ret < 0) {
            RoboAct('W', 60); 
        } else if ( ret == 0 ) { // Action......
            ;
        } else{
            printf("T1ActRoboAll-StationInVision step4, recognition failed, try walking forward 60CM to adjust the position.\n");
            WorkStatusStaIn = 5;
        }
    } else if (WorkStatusStaIn == 5){ // turn round to face station
        ret = RoboAct('=',0);
        if (ret < 0) {
            //RoboAct('Z', 90);
            RoboAct(meCmdZ, 90);
        } else if ( ret == 0 ) { // Action......
            ;
        } else{
            if(JudgeFaceNew() == -1000){ // didn't detect charging pile yet
                printf("T1ActRoboAll-StationInVision step5, Charging pile still not detected, continue adjusting position\n");
                WorkStatusStaIn = 6;
            }else{ // detect charging pile successgully 
                printf("T1ActRoboAll-StationInVision step5, Detected a charging pile\n");
                WorkStatusStaIn = 1;
            }
        }
    } else if(WorkStatusStaIn == 6){ // turn left 90
        ret = RoboAct('=',0);
        if (ret < 0) {
            //RoboAct('Z', 90);
            RoboAct(meCmdZ, 90);
        }else if ( ret == 0 ) { // Action......
            ;
        }else{
            printf("T1ActRoboAll-StationInVision step6, recognition failed, try turning left to adjust the position.\n");
            WorkStatusStaIn = 7;
        }
    } else if(WorkStatusStaIn == 7){ //walkforward 120cm
        ret = RoboAct('=',0);
        if (ret < 0) {
            RoboAct('W', 120); 
        }else if ( ret == 0 ) { // Action......
            ;
        }else{
            printf("T1ActRoboAll-StationInVision step7, recognition failed, try walking forward 120CM to adjust the position.\n");
            WorkStatusStaIn = 8;
        }
    } else if(WorkStatusStaIn == 8){ // turn round to face station
        ret = RoboAct('=',0);
        if (ret < 0) {
            //RoboAct('C', 90);
            RoboAct(meCmdC, 90);
        } else if ( ret == 0 ) { // Action......
            ;
        } else{
            if(JudgeFaceNew() == -1000){
                printf("T1ActRoboAll-StationInVision step8, stop pile still not detected, exit\n");
                WorkStatusStaIn = -1;
            }else{
                printf("T1ActRoboAll-StationInVision step8, Detected a charging pile\n");
                WorkStatusStaIn = 1;
            }
        }
    } else if ( WorkStatusStaIn == 9 ) { // adjust position
        int distance = 90 - meVisionDistY;
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            if ( distance > 0 ){
                RoboAct('S', distance+10);
            } else {
                RoboAct('W', abs(distance)+10);
            }
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step9, adjust y position done.\n");
            WorkStatusStaIn = 1;
        }
    } else if ( WorkStatusStaIn == 10 ) { // turn round
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            if ( meVisionDistX > 35 ){
                printf("T1ActRoboAll-StationInVision step10, turn left to adjust x position.\n");
                //RoboAct('C',90);
                RoboAct(meCmdC, 90);
            } else {
                printf("T1ActRoboAll-StationInVision step10, turn right to adjust x position.\n");
                //RoboAct('Z',90);
                RoboAct(meCmdZ, 90);
            }
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step10, turn round done.\n");
            WorkStatusStaIn = 11;
        }
    } else if ( WorkStatusStaIn == 11 ) { // adjust x position
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            RoboAct('W', abs(meVisionDistX)+8);
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step11, walk forward %f cm to adjust x position.\n",abs(meVisionDistX));
            WorkStatusStaIn = 1;
        }
    } else if ( WorkStatusStaIn == 12 ) { // adjust x position
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            if ( meVisionDistX > 0 ){
                printf("T1ActRoboAll-StationInVision step12, turn left to adjust x position.\n");
                //RoboAct('C',90);
                RoboAct(meCmdC, 90);
            } else {
                printf("T1ActRoboAll-StationInVision step12, turn right to adjust x position.\n");
                //RoboAct('Z',90);
                RoboAct(meCmdZ, 90);
            }
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step12, turn round done.\n");
            WorkStatusStaIn = 11;
        }
    } else if ( WorkStatusStaIn == 13 ) { // walkforward to back station
        int distance = meVisionDistY - 20;
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            RoboAct('w',abs(distance));
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step13, walk forward %dcm\n",distance);
            WorkStatusStaIn = 14;
        }
    } else if ( WorkStatusStaIn == 14 ) { // check if need adjust position
        int res = JudgeFaceNew();
        if ( abs(meVisionYawAngle) <= 3 || res == -1000 ) { //meVisionDistX <= 2 && meVisionDistX >= -6 || can't see april tag
            printf("T1ActRoboAll-StationInVision step14, prepare to Final adjustment done\n");
            WorkStatusStaIn = 15;
        } else {
            printf("T1ActRoboAll-StationInVision step14, prepare to Final adjustment done\n");
            WorkStatusStaIn = 18;
        }
    } else if ( WorkStatusStaIn == 15 ) { // check if need adjust position
          ret = RoboAct('=',0);
            if ( ret < 0 ) {
                RoboAct('r',200);
            } else if ( ret == 0 ) { // Action......
                ;
            } else {
                WorkStatusStaIn = 16;
                printf("T1ActRoboAll-StationInVision step15, Final adjustment done\n");
            }
    } else if ( WorkStatusStaIn == 16 ) { // check if need adjust position
        if( JudgeCharge() == 1 ) {
            printf("T1ActRoboAll-StationInVision step16, StationInVision done\n");
            mevisEnterCnt = 0;
            FlagInCharge = 0;

            WorkStatusStaIn = -1;
        } else {
            printf("T1ActRoboAll-StationInVision step16, No charging detected\n");
            WorkStatusStaIn = 17;
        }
    } else if ( WorkStatusStaIn == 17 ) { // check if need adjust position
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            RoboAct('S',100);
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step17, Pile insertion failed, withdraw 100cm and retry\n");
            mevisEnterCnt = 0;
            WorkStatusStaIn = 1;
        }
    } else if ( WorkStatusStaIn == 18 ) { // adujust yaw
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            if ( meVisionYawAngle > 3 ){
                //RoboAct('Z',abs(meVisionYawAngle));
                RoboAct(meCmdZ,abs(meVisionYawAngle));
            } else if ( meVisionYawAngle < -3 ) {
                //RoboAct('C',abs(meVisionYawAngle));
                RoboAct(meCmdC,abs(meVisionYawAngle));
            }
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step18, turn round %d\n",meVisionYawAngle);
            WorkStatusStaIn = 14;
        }
    } else if ( WorkStatusStaIn == 19 ) { // adujust yaw
            ret = RoboAct('=',0);
            if ( ret < 0 ) {
                if (meVisionYawAngle > 0){
                    //RoboAct('C', abs(meVisionYawAngle));
                    RoboAct(meCmdC,abs(meVisionYawAngle));
                } else {
                    //RoboAct('Z', abs(meVisionYawAngle));
                    RoboAct(meCmdZ,abs(meVisionYawAngle));
                }
            } else if ( ret == 0 ) { // Action......
                ;
            } else {
                printf("T1ActRoboAll-StationInVision step19, Adjust meVisionYawAngle : %d.\n",meVisionYawAngle);
                WorkStatusStaIn = 1;
                meAngleStation -= meVisionYawAngle;
            }
    }
    return WorkStatusStaIn;
}
/*
 * StationInVision - Robot auto return to charging pile
 * Return: Current process status (-1=Finish/Error, others=in progress)
 * 2026.05.14: created by Xs
 */
int T1ActRoboAll::StationInVisionXS() {
    int ret;
    static int count = 0;
    if ( WorkStatusStaIn < 0 || FlagPause == 1) { // Stopped or paused
        return WorkStatusStaIn;
    } else if ( WorkStatusStaIn == 0 ) {// Initial state
        if( IsCharing == 1 ){
            printf("T1ActRoboAll-StationInVision step0, Initial state\n");
            // pmyVision->SetAprilTagSize(GetAprilTagSize(1));
            WorkStatusStaIn = 1;
            IsCharing = 0;
            strcpy(meStrSendVis,"0"); // range: 0--9, default is 9, 0 is always high resolution         WorkStatusStaOut = 1;
        }else{
            printf("T1ActRoboAll-StationInVision step0, don't need Initial state\n");
            // pmyVision->SetAprilTagSize(GetAprilTagSize(1));
            WorkStatusStaIn = 1;
            strcpy(meStrSendVis,"0"); // range: 0--9, default is 9, 0 is always high resolution         WorkStatusStaOut = 1;
        }     
    } else if ( WorkStatusStaIn == 1) { // turn to face charge station
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            printf("T1ActRoboAll-StationInVision step1, Turning towards charge station\n");
            int diff = ((meAngleStation + myDiffMI.yaw - myRoboM.yaw + 540) % 360) - 180;
            if ( diff > 0 ){
                //RoboAct('C', diff);
                RoboAct(meCmdC, diff);
            } else {
                //RoboAct('Z', abs(diff));
                RoboAct(meCmdZ, abs(diff));
            }
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            WorkStatusStaIn=21;
            meRotationStableStart=0;
            count=0;
            printf("T1ActRoboAll-StationInVision step2, turn to face charge station success.\n");
        }
    } else if ( WorkStatusStaIn==21 ) { // wait until the robot stops rotating
        if ( IsRotationStable(0.5f, 200) ) {
            WorkStatusStaIn=2;
            count=0;
            printf("T1ActRoboAll-StationInVision step21, robot rotation stable, start AprilTag detection.\n");
        }
    } else if ( WorkStatusStaIn == 2 ) { // detect the april tag
        int res=JudgeFaceNew();
        //printf("T1ActRoboAll-StationInVision step2, meVisionDistX-Y-Angle=(%f,%f,%d)\n", meVisionDistX, meVisionDistY, meVisionYawAngle);
        if ( res==-1000 ) {
            printf("T1ActRoboAll-StationInVision step2, not detected, search for pile.\n");
            WorkStatusStaIn=3;
        } else if ( meVisionDistY>110 || meVisionDistY<80 ) {
            printf("T1ActRoboAll-StationInVision step2, Y distance out of range: %.1f.\n", meVisionDistY);
            WorkStatusStaIn=9;
        } else if ( meVisionDistX>10 || meVisionDistX<-10 ) {
            printf("T1ActRoboAll-StationInVision step2, X distance out of range: %.1f.\n", meVisionDistX);
            WorkStatusStaIn=10;
        } else if ( abs(meVisionYawAngle)>2 ) {
            printf("T1ActRoboAll-StationInVision step2, yaw angle too large: %d.\n", meVisionYawAngle);
            WorkStatusStaIn=19;
        } else {
            printf("T1ActRoboAll-StationInVision step2, aligned successfully.\n");
            WorkStatusStaIn=13;
        }
    } else if (WorkStatusStaIn == 3) {
        ret = RoboAct('=',0);
        if (ret < 0) {
            //RoboAct('C', 90);
            RoboAct(meCmdC, 90);
        }else if ( ret == 0 ) { // Action......
            ;
        }else{
            printf("T1ActRoboAll-StationInVision step3, Adjustment turn right 90° done\n");
            WorkStatusStaIn = 4;
        }
    } else if(WorkStatusStaIn == 4){ // walkforward 60cm
        ret = RoboAct('=',0);
        if (ret < 0) {
            RoboAct('W', 20); 
        } else if ( ret == 0 ) { // Action......
            ;
        } else{
            printf("T1ActRoboAll-StationInVision step4, recognition failed, try walking forward 20CM to adjust the position.\n");
            WorkStatusStaIn = 5;
        }
    } else if (WorkStatusStaIn == 5){ // turn round to face station
        ret = RoboAct('=',0);
        if (ret < 0) {
            //RoboAct('Z', 90);
            RoboAct(meCmdZ, 90);
        } else if ( ret == 0 ) { // Action......
            ;
        } else{
            if(JudgeFaceNew() == -1000){ // didn't detect charging pile yet
                printf("T1ActRoboAll-StationInVision step5, Charging pile still not detected, continue adjusting position\n");
                WorkStatusStaIn = 6;
            }else{ // detect charging pile successgully 
                printf("T1ActRoboAll-StationInVision step5, Detected a charging pile\n");
                WorkStatusStaIn = 1;
            }
        }
    } else if(WorkStatusStaIn == 6){ // turn left 90
        ret = RoboAct('=',0);
        if (ret < 0) {
            //RoboAct('Z', 90);
            RoboAct(meCmdZ, 90);
        }else if ( ret == 0 ) { // Action......
            ;
        }else{
            printf("T1ActRoboAll-StationInVision step6, recognition failed, try turning left to adjust the position.\n");
            WorkStatusStaIn = 7;
        }
    } else if(WorkStatusStaIn == 7){ //walkforward 120cm
        ret = RoboAct('=',0);
        if (ret < 0) {
            RoboAct('W', 20); 
        }else if ( ret == 0 ) { // Action......
            ;
        }else{
            printf("T1ActRoboAll-StationInVision step7, recognition failed, try walking forward 20CM to adjust the position.\n");
            WorkStatusStaIn = 1;
        }
    }else if ( WorkStatusStaIn == 9 ) { // adjust position
        int distance = meVisionDistY - 90;
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            if ( distance < 0 ){
                RoboAct('S', abs(distance)+10);
            } else {
                RoboAct('W', distance+10);
            } 
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step9, adjust y position done.\n");
            WorkStatusStaIn = 1;
        }
    } else if ( WorkStatusStaIn == 10 ) { // turn round
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            if ( meVisionDistX > 0 ){
                printf("T1ActRoboAll-StationInVision step10, turn left to adjust x position.\n");
                //RoboAct('C',90);
                RoboAct(meCmdC, 90);
            } else {
                printf("T1ActRoboAll-StationInVision step10, turn right to adjust x position.\n");
                //RoboAct('Z',90);
                 RoboAct(meCmdZ, 90);
            }
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step10, turn round done.\n");
            WorkStatusStaIn = 11;
        }
    } else if ( WorkStatusStaIn == 11 ) { // adjust x position
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            RoboAct('W', abs(meVisionDistX)+10);
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step11, walk forward %f cm to adjust x position.\n",abs(meVisionDistX));
            WorkStatusStaIn = 1;
        }
    } else if ( WorkStatusStaIn == 13 ) { // walkforward to back station
        int distance = meVisionDistY - 65;
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            RoboAct('w',abs(distance)+10);
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step13, walk forward %d cm\n",distance);
            WorkStatusStaIn=14;
            meRotationStableStart=0;
        }
    } else if ( WorkStatusStaIn == 14 ) { // check if need adjust position
        if ( IsRotationStable(0.5f, 200) ) {
            int res=JudgeFaceNew();
            if ( res==-1000 ) {
                printf("T1ActRoboAll-StationInVision step14, AprilTag not detected, search for pile.\n");
                WorkStatusStaIn=3;
            } else if ( abs(meVisionYawAngle)>2 ) {
                printf("T1ActRoboAll-StationInVision step14, adjust yaw angle: %d.\n", meVisionYawAngle);
                WorkStatusStaIn=19;
            } else if ( abs(meVisionDistX)>3 ) {
                printf("T1ActRoboAll-StationInVision step14, adjust X position: %.1f.\n", meVisionDistX);
                WorkStatusStaIn=18;
            } else {
                printf("T1ActRoboAll-StationInVision step14, alignment completed.\n");
                WorkStatusStaIn=15;
            }
        }

    } else if ( WorkStatusStaIn == 15 ) { // check if need adjust position
          ret = RoboAct('=',0);
            if ( ret < 0 ) {
                RoboAct('r',150);
            } else if ( ret == 0 ) { // Action......
                ;
            } else {
                WorkStatusStaIn = 16;
                printf("T1ActRoboAll-StationInVision step15, Final adjustment done\n");
            }
    } else if ( WorkStatusStaIn == 16 ) { // check if need adjust position
        if( JudgeCharge() == 1 ) {
            printf("T1ActRoboAll-StationInVision step16, StationInVision done\n");
            mevisEnterCnt = 0;
            FlagInCharge = 0;
            strcpy(meStrSendVis,"9"); // range: 0--9, default is 9, 0 is always high resolution
            WorkStatusStaIn = -1;
        } else {
            printf("T1ActRoboAll-StationInVision step16, No charging detected\n");
            WorkStatusStaIn = 17;
        }
    } else if ( WorkStatusStaIn == 17 ) { // check if need adjust position
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            RoboAct('S',100);
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step17, Pile insertion failed, withdraw 100cm and retry\n");
            mevisEnterCnt = 0;
            WorkStatusStaIn = 1;
        }
    } else if ( WorkStatusStaIn == 18 ) { // adujust yaw
        targetYaw = meVisionDistX * 0.8f;
        if (targetYaw > 8.0f || targetYaw < -8.0f) {
            WorkStatusStaIn = 1;
        } else {
            WorkStatusStaIn = 20;
        }   
    } else if ( WorkStatusStaIn == 19 ) { // adujust yaw
        ret=RoboAct('=',0);
        if ( ret < 0 ) {
            if ( meVisionYawAngle>0 ) {
                RoboAct('C', 1);
                //RoboAct('C',1);
            } else {
                RoboAct('Z', 1);
                //RoboAct('Z',1);
            }
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            WorkStatusStaIn=21;
            meRotationStableStart=0;
            printf("T1ActRoboAll-StationInVision step19, yaw adjustment done, wait for rotation stable.\n");
        }
    }else if ( WorkStatusStaIn == 20 ) { // adujust yaw
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            if (targetYaw > 0){
                RoboAct('C', abs(targetYaw ));
                //RoboAct('C',1);
            } else {
                RoboAct('Z', abs(targetYaw ));
                //RoboAct('Z',1);
            }
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step20, Adjust targetYaw : %f.\n",targetYaw );
            WorkStatusStaIn = 15;
        }
    }
    return WorkStatusStaIn;
}
/*
 * StationInVision - Robot auto return to charging pile
 * Return: Current process status (-1=Finish/Error, others=in progress)
 * 2026.05.14: created by Xs
 */
/*
int T1ActRoboAll::StationInVisionXS() {
    int ret;
    static int count = 0;
    if ( WorkStatusStaIn < 0 || FlagPause == 1) { // Stopped or paused
        return WorkStatusStaIn;
    } else if ( WorkStatusStaIn == 0 ) {// Initial state
        if( IsCharing == 1 ){
            printf("T1ActRoboAll-StationInVision step0, Initial state\n");
            // pmyVision->SetAprilTagSize(GetAprilTagSize(1));
            WorkStatusStaIn = 1;
            IsCharing = 0;
            strcpy(meStrSendVis,"0"); // range: 0--9, default is 9, 0 is always high resolution         WorkStatusStaOut = 1;
        }else{
            printf("T1ActRoboAll-StationInVision step0, don't need Initial state\n");
            // pmyVision->SetAprilTagSize(GetAprilTagSize(1));
            WorkStatusStaIn = 1;
            strcpy(meStrSendVis,"0"); // range: 0--9, default is 9, 0 is always high resolution         WorkStatusStaOut = 1;
        }     
    } else if ( WorkStatusStaIn == 1) { // turn to face charge station
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            printf("T1ActRoboAll-StationInVision step1, Turning towards charge station\n");
            int diff = ((meAngleStation + myDiffMI.yaw - myRoboM.yaw + 540) % 360) - 180;
            if ( diff > 0 ){
                //RoboAct('C', diff);
                RoboAct(meCmdC, diff);
            } else {
                //RoboAct('Z', abs(diff));
                RoboAct(meCmdZ, abs(diff));
            }
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            WorkStatusStaIn = 2;
            count = 0;
            printf("T1ActRoboAll-StationInVision step2, turn to face charge station success.\n");
        }
    } else if ( WorkStatusStaIn == 2 ) { // detect the april tag
        count++;
        if(count >= 100){
            count = 0;
            int res = JudgeFaceNew();
            //printf("T1ActRoboAll-StationInVision step2, meVisionDistX-Y-Angle=(%f,%f,%d)\n", meVisionDistX, meVisionDistY, meVisionYawAngle);
            WorkStatusStaIn = ProcessVisionResult(res);
        }
    } else if (WorkStatusStaIn == 3) {
        ret = RoboAct('=',0);
        if (ret < 0) {
            //RoboAct('C', 90);
            RoboAct(meCmdC, 90);
        }else if ( ret == 0 ) { // Action......
            ;
        }else{
            printf("T1ActRoboAll-StationInVision step3, Adjustment turn right 90° done\n");
            WorkStatusStaIn = 4;
        }
    } else if(WorkStatusStaIn == 4){ // walkforward 60cm
        ret = RoboAct('=',0);
        if (ret < 0) {
            RoboAct('W', 20); 
        } else if ( ret == 0 ) { // Action......
            ;
        } else{
            printf("T1ActRoboAll-StationInVision step4, recognition failed, try walking forward 20CM to adjust the position.\n");
            WorkStatusStaIn = 5;
        }
    } else if (WorkStatusStaIn == 5){ // turn round to face station
        ret = RoboAct('=',0);
        if (ret < 0) {
            //RoboAct('Z', 90);
            RoboAct(meCmdZ, 90);
        } else if ( ret == 0 ) { // Action......
            ;
        } else{
            if(JudgeFaceNew() == -1000){ // didn't detect charging pile yet
                printf("T1ActRoboAll-StationInVision step5, Charging pile still not detected, continue adjusting position\n");
                WorkStatusStaIn = 6;
            }else{ // detect charging pile successgully 
                printf("T1ActRoboAll-StationInVision step5, Detected a charging pile\n");
                WorkStatusStaIn = 1;
            }
        }
    } else if(WorkStatusStaIn == 6){ // turn left 90
        ret = RoboAct('=',0);
        if (ret < 0) {
            //RoboAct('Z', 90);
            RoboAct(meCmdZ, 90);
        }else if ( ret == 0 ) { // Action......
            ;
        }else{
            printf("T1ActRoboAll-StationInVision step6, recognition failed, try turning left to adjust the position.\n");
            WorkStatusStaIn = 7;
        }
    } else if(WorkStatusStaIn == 7){ //walkforward 120cm
        ret = RoboAct('=',0);
        if (ret < 0) {
            RoboAct('W', 20); 
        }else if ( ret == 0 ) { // Action......
            ;
        }else{
            printf("T1ActRoboAll-StationInVision step7, recognition failed, try walking forward 20CM to adjust the position.\n");
            WorkStatusStaIn = 1;
        }
    }else if ( WorkStatusStaIn == 9 ) { // adjust position
        int distance = meVisionDistY - 90;
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            if ( distance < 0 ){
                RoboAct('S', abs(distance)+10);
            } else {
                RoboAct('W', distance+10);
            } 
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step9, adjust y position done.\n");
            WorkStatusStaIn = 1;
        }
    } else if ( WorkStatusStaIn == 10 ) { // turn round
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            if ( meVisionDistX > 0 ){
                printf("T1ActRoboAll-StationInVision step10, turn left to adjust x position.\n");
                //RoboAct('C',90);
                RoboAct(meCmdC, 90);
            } else {
                printf("T1ActRoboAll-StationInVision step10, turn right to adjust x position.\n");
                //RoboAct('Z',90);
                 RoboAct(meCmdZ, 90);
            }
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step10, turn round done.\n");
            WorkStatusStaIn = 11;
        }
    } else if ( WorkStatusStaIn == 11 ) { // adjust x position
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            RoboAct('W', abs(meVisionDistX)+10);
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step11, walk forward %f cm to adjust x position.\n",abs(meVisionDistX));
            WorkStatusStaIn = 1;
        }
    } else if ( WorkStatusStaIn == 13 ) { // walkforward to back station
        int distance = meVisionDistY - 65;
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            RoboAct('w',abs(distance)+10);
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step13, walk forward %d cm\n",distance);
            WorkStatusStaIn = 14;
        }
    } else if ( WorkStatusStaIn == 14 ) { // check if need adjust position
        count++;
        if(count >= 200){
        count = 0;
        int res = JudgeFaceNew();
            if ( abs(meVisionYawAngle) <= 2 && abs(meVisionDistX) <= 3 ) { //meVisionDistX <= 2 && meVisionDistX >= -6 || can't see april tag
                printf("T1ActRoboAll-StationInVision step14, prepare to Final adjustment done\n");
                WorkStatusStaIn = 15;
            } else if (abs(meVisionDistX) <= 10 && abs(meVisionYawAngle) <=2 ) { // need adjust position
                WorkStatusStaIn = 18;
                printf("T1ActRoboAll-StationInVision step18, adjust x position\n");
            } else {
                printf("T1ActRoboAll-StationInVision step14, prepare to Final adjustment done\n");
                WorkStatusStaIn = 19;
            }
        }

    } else if ( WorkStatusStaIn == 15 ) { // check if need adjust position
          ret = RoboAct('=',0);
            if ( ret < 0 ) {
                RoboAct('r',150);
            } else if ( ret == 0 ) { // Action......
                ;
            } else {
                WorkStatusStaIn = 16;
                printf("T1ActRoboAll-StationInVision step15, Final adjustment done\n");
            }
    } else if ( WorkStatusStaIn == 16 ) { // check if need adjust position
        if( JudgeCharge() == 1 ) {
            printf("T1ActRoboAll-StationInVision step16, StationInVision done\n");
            mevisEnterCnt = 0;
            FlagInCharge = 0;
            strcpy(meStrSendVis,"9"); // range: 0--9, default is 9, 0 is always high resolution
            WorkStatusStaIn = -1;
        } else {
            printf("T1ActRoboAll-StationInVision step16, No charging detected\n");
            WorkStatusStaIn = 17;
        }
    } else if ( WorkStatusStaIn == 17 ) { // check if need adjust position
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            RoboAct('S',100);
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step17, Pile insertion failed, withdraw 100cm and retry\n");
            mevisEnterCnt = 0;
            WorkStatusStaIn = 1;
        }
    } else if ( WorkStatusStaIn == 18 ) { // adujust yaw
        targetYaw = meVisionDistX * 0.8f;
        if (targetYaw > 8.0f || targetYaw < -8.0f) {
            WorkStatusStaIn = 1;
        } else {
            WorkStatusStaIn = 20;
        }   
    } else if ( WorkStatusStaIn == 19 ) { // adujust yaw
        ret = RoboAct('=',0);
        int res = JudgeFaceNew();
        if ( ret < 0 ) {
            if (meVisionYawAngle > 0){
                RoboAct('Z', 1);
                //RoboAct('C',1);
            } else {
                RoboAct('C', 1);
                //RoboAct('Z',1);
            }
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            if (abs(meVisionYawAngle) >= 2) {
                WorkStatusStaIn = 19;
            } else {
                WorkStatusStaIn = 1;
                meAngleStation -= meVisionYawAngle;
            }
        }
    }else if ( WorkStatusStaIn == 20 ) { // adujust yaw
        ret = RoboAct('=',0);
        if ( ret < 0 ) {
            if (targetYaw > 0){
                RoboAct('C', abs(targetYaw ));
                //RoboAct('C',1);
            } else {
                RoboAct('Z', abs(targetYaw ));
                //RoboAct('Z',1);
            }
        } else if ( ret == 0 ) { // Action......
            ;
        } else {
            printf("T1ActRoboAll-StationInVision step20, Adjust targetYaw : %f.\n",targetYaw );
            WorkStatusStaIn = 15;
        }
    }
    return WorkStatusStaIn;
}
*/
/** save the current gridmap before gohome()
 * 2026.02.27: ported to real robot by Xs
 */
void T1ActRoboAll::GoHomeGridMapSave() {
    printf("T1ActRoboAll-GoHomeGridMapSave begin.\n");
}

/** read the gridmap saved when gohome()
 * 2026.02.27: ported to real robot by Xs
 */
void T1ActRoboAll::GoHomeGridMapRead() {
    printf("T1ActRoboAll-GoHomeGridMapRead begin.\n");
}

/** clear the saved point when gohome()
 * 2026.02.24: created by Xs
 */ 
int T1ActRoboAll::GoHomeClearPoint() {
    printf("T1ActRoboAll-GoHomeClearPoint begin.\n");
    char tmpstr[200];
    sprintf(tmpstr,"../Tsk/GohomePoint.dat");
    std::ofstream fileout(tmpstr);
    if (!fileout.is_open()) {
        T1Utils::LogBug("T1ActRoboAll-GoHomeClearPoint, Can not open ../Tsk/GohomePoint.dat");
        return -1;
    }
    fileout << "0,nomap,0,0,0,0,0,0" << std::endl;
    fileout.close();
    return 0;
}

/** save the current point before gohome()
 * 2026.02.27: created by Xs
 */ 
int T1ActRoboAll::GoHomeSavePoint() {
    printf("T1ActRoboAll-GoHomeSavePoint begin.\n");
    char tmpstr[200];
    sprintf(tmpstr,"../Tsk/GohomePoint.dat");
    std::ofstream fileout(tmpstr);
    if (!fileout.is_open()) {
        T1Utils::LogBug("T1ActRoboAll-GoHomeSavePoint, Can not open ../Tsk/GohomePoint.dat");
        return -1;
    }
    fileout << "1," << pMap->MapSelectRead() << ",";
    fileout << ((T1Map2 *)pMap)->GetLength('x') << "," << ((T1Map2 *)pMap)->GetLength('y') << ",";
    fileout << myRoboM.fx << "," << myRoboM.fy << "," << meWorkline << "," << flagReverse << "," << std::endl;
    fileout.close();
    ((T1Map2 *)pMap)->GridMapSave("../Tsk/GoHomeGridMap.dat");
    printf("T1ActRoboAll-GoHomeSavePoint saved to %s\n", tmpstr);
    return 0;
}

/** read the saved point when gohome()
 * return:  0,no saved pint         1,get saved point
 * 2026.02.24: created by Xs
 */ 
int T1ActRoboAll::GoHomeReadPoint() {
    //printf("T1ActRoboAll-GoHomeReadPoint begin.\n");
    int flagSaved = 0;
    char tmpstr[200];
    sprintf(tmpstr,"../Tsk/GohomePoint.dat");
    std::ifstream filein(tmpstr);
    if (!filein.is_open()) {
        T1Utils::LogBug("T1ActRoboAll-GoHomeReadPoint, Can not open ../Tsk/GohomePoint.dat");
        return flagSaved;
    }
    
    std::string line;
    std::vector<std::string> list1;
    while ( std::getline(filein,line) ) {
        list1 = T1Utils::split(line,',');
        //printf("T1ActRoboAll-GoHomeReadPoint step1, length=%ld.\n", list1.size());
        if ( list1.size() != 8 ) { // if you add some fields, notice the GoHomeClearPoint();
            break;
        }
        if ( line[0] == '1' ) {
            std::string mapselect = pMap->MapSelectRead();
            //printf("T1ActRoboAll-GoHomeReadPoint step2, [%s],(%s).\n", list1[1].c_str(), mapselect.c_str());
            if ( list1[1] == mapselect  // select map changed
                && ((T1Map2 *)pMap)->GetLength('x') == std::stoi(list1[2]) // grid map size error
                && ((T1Map2 *)pMap)->GetLength('y') == std::stoi(list1[3]) ) {
                printf("T1ActRoboAll-GoHomeReadPoint step3, saved map restored.\n");
                flagSaved = 1;
                GoHomePoint.fx = std::stod(list1[4]);
                GoHomePoint.fy = std::stod(list1[5]);
                meWorkline = std::stoi(list1[6]);
                flagReverse = std::stoi(list1[7]);
                ((T1Map2 *)pMap)->GridMapRead("../Tsk/GoHomeGridMap.dat");
                break; // only getline 1 line
            } else {
                printf("T1ActRoboAll-GoHomeReadPoint step4, saved map don't match.\n");
            }
        }
    }
    filein.close();
    //printf("T1ActRoboAll-GoHomeReadPoint end. read(%s)\n", line.c_str());
    printf("T1ActRoboAll-GoHomeReadPoint end. read(%d,%s,%.1f,%.1f,%d,%d,%d,%d)\n", flagSaved, list1[1].c_str(), GoHomePoint.fx, GoHomePoint.fy, meWorkline, flagReverse, meWorkline, flagReverse);
    return flagSaved;
}

/** go back to point 0,0 alone Astar path
 * 2026.02.24: created by Leo
 ** fixed GoHome not returning home - changed findAstarToPoint param from myMower6 to myRoboM added WorkStatusRobo=0 in state 1 to ensure Work2() executes properly
 *  2026.02.26: reated by Xs
 */
int T1ActRoboAll::GoHome() {
    if ( WorkStatusGH < 0 || FlagPause == 1 ) {
        ;//return WorkStatusGH;
    } else if ( WorkStatusGH == 0 ) {
        //printf("T1ActRoboAll-GoHome[0]: prepare to go home using A* path. myRoboM(%.1f,%.1f)\n", myRoboM.fx, myRoboM.fy);
        RoboAct('g', 0);
        int endX = 0, endY = 0;
        if (((T1Map2 *)pMap)->findAstarToPoint((int)myRoboM.fx, (int)myRoboM.fy, endX, endY) < 0) {
            T1Utils::LogBug("T1ActRoboAll-GoHome[0]: error, findAstarToPoint() failed.\n");
            WorkStatusGH = -1;
        } else {
            printf("T1ActRoboAll-GoHome[0]: from(%.1f,%.1f), Astar path length=%d\n", myRoboM.fx, myRoboM.fy, pMap->GetLength('2'));
            WorkStatusGH = 1;
            WorkStatusRobo = 0;
        }
    } else if (WorkStatusGH == 1) {
        int retWork = Work2();
        if (retWork < 0) {
            printf("T1ActRoboAll-GoHome[1]: work2() completed, reach (0,0).\n");
            WorkStatusGH = -1;
        }
    } else {
        char bugMsg[128];
        sprintf(bugMsg, "T1ActRoboAll-GoHome: ERROR unknown WorkStatusGH=%d", WorkStatusGH);
        T1Utils::LogBug(bugMsg);
        WorkStatusGH = -1;
        RoboAct('g', 0);
    }
    return WorkStatusGH;
}

/* Cut along the border
 * 2026.04.15: created by LeoF
 * 2026.06.24: modified by BY
 */
int T1ActRoboAll::GoBorder() {
    static MapPoint nextPoint;
    if ( WorkStatusGB<0 || FlagPause==1 ) {
        return WorkStatusGB;
    } else if ( WorkStatusGB==0 ) {
        printf("T1ActRoboAll-GoBorder step0: prepare to go border using A* path. myRoboM(%.1f,%.1f)\n", myRoboM.fx, myRoboM.fy);
        RoboAct('g', 0);
        if (((T1Map2 *)pMap)->GetRouteBorder() < 0) {
            T1Utils::LogBug("T1ActRoboAll-GoBorder[0]: error, findAstarToPoint() failed.\n");
            WorkStatusGB = -1;
        } else {
            //printf("T1ActRoboAll-GoBorder[0]: get the RoutePlan2.length=%d\n", pMap->GetLength('2'));
            WorkStatusGB = 1;
        }
    } else if ( WorkStatusGB==1 ) {
        meWorklength = pMap->GetLength('P'); meWorkline = 0; meWorkstep = 0;
        MapPoint nextPoint = pMap->GetPoint('P',meWorkline);
        myRoboM.fx = nextPoint.lon;  myRoboM.fy = nextPoint.lat;
        meWorkline++;
        WorkStatusGB = 2;
        printf("T1ActRoboAll-GoBorder step1: prepare to workall lines. workline=%d\n", meWorkline );
    } else if ( WorkStatusGB==2 ) {
        nextPoint = pMap->GetPoint('P',meWorkline);
        ((T1Map2 *)pMap)->findAstarToPoint((int)myRoboM.fx, (int)myRoboM.fy, nextPoint.lon, nextPoint.lat);
        printf("T1ActRoboAll-GoBorder step2: prepare find Astar route. workline=%d, now=(%.f,%.f), target=(%lld,%lld)\n",
            meWorkline, myRoboM.fx, myRoboM.fy, nextPoint.lon, nextPoint.lat );
        WorkStatusRobo = 0; // WorkStatus to control Work2();
        WorkStatusGB = 3;
    } else if ( WorkStatusGB==3 ) {
        int retWork = Work2();
        if (retWork < 0) {
            printf("T1ActRoboAll-GoBorder step3: work2() completed. workline=%d \n", meWorkline);
            myRoboM.fx = nextPoint.lon;  myRoboM.fy = nextPoint.lat;
            meWorkline++;
            if ( meWorkline>=meWorklength )
                WorkStatusGB = -1;
            else
                WorkStatusGB = 2;
        }
    } else {
        char bugMsg[128];
        snprintf(bugMsg, sizeof(bugMsg), "T1ActRoboAll-GoBorder: ERROR unknown WorkStatusGB=%d", WorkStatusGB);
        T1Utils::LogBug(bugMsg);
        WorkStatusGB = -1;
        RoboAct('g', 0);
    }
    return WorkStatusGB;
}


/* work on RoutePlan2, and use the T1Map2 class
 * return==1: Work,  return<0: finished work,
 */
int T1ActRoboAll::Work2() {
    int debug=0;
    //printf("T1ActRoboAll-Work2 begin: WS6=%d, (%d,%d,%d), Min(%lld,%lld), Max(%lld,%lld)\n", WorkStatusRobo, meWorklength, meWorkline, meWorkstep, pMap->GetPoint('i',0).lon, pMap->GetPoint('i',0).lat, pMap->GetPoint('a',0).lon, pMap->GetPoint('a',0).lat);
    MapPoint tmpPoint;
    if ( WorkStatusRobo < 0 || FlagPause == 1) {
        ;
    } else if ( WorkStatusRobo == 0 ) {
        //RoboAct('I',195);
        //pMap->SetRouSet(meFact.StepLinex,meFact.StepLiney);
        //pMap->GetRoute(0);
        meWorklength2 = pMap->GetLength('2'); meWorkline2 = 0; meWorkstep = 0;
        WorkStatusRobo = 1;
        //sprintf(StrCmdSend, "@CMD,n\r\n");
        //myMower6.fx = 0;  myMower6.fy = 0;
        if (debug>1) printf("T1ActRoboAll-Work2 step1: (%d,%d,%d), WS6=%d, Min:(%lld,%lld), Max:(%lld,%lld), myDiffMI=%d\n", meWorklength2, meWorkline2, meWorkstep, WorkStatusRobo, pMap->GetPoint('i',0).lon, pMap->GetPoint('i',0).lat, pMap->GetPoint('a',0).lon, pMap->GetPoint('a',0).lat, myDiffMI.yaw);
    } else { 
        //printf("T1ActRoboAll-Work2 step2: (%d,%d,%d), RSW=%d, Min:(%lld,%lld), Max:(%lld,%lld)\n", meWorklength, meWorkline, meWorkstep, WorkStatusRobo, pMap->GetPoint('i',0).lon, pMap->GetPoint('i',0).lat, pMap->GetPoint('a',0).lon, pMap->GetPoint('a',0).lat);
        if ( meWorkline2 >= meWorklength2 ) {
            //printf("T1ActRoboAll-Work2 step3, scaned all lines: [%d,meWorkline=(%d,%d)], RSW=%d\n", meWorklength, meWorkline, meWorkstep, WorkStatusRobo);
            WorkStatusRobo = -1; // just do once
        } else {
            int ret = WorkLine(((T1Map2 *)pMap)->GetPoint2('2',meWorkline2));
            //printf("T1ActRoboAll-Work2 step4: (%d,%d,%d), WS6=%d, ret=%d\n", meWorklength2, meWorkline2, meWorkstep, WorkStatusRobo, ret);
            if ( ret == 0 ) {
                meWorkline2++;
                if (debug>1) printf("T1ActRoboAll-Work2 step4a, scaned one lines: (%d,%d,%d), RSW=%d, ", meWorklength2, meWorkline2, meWorkstep, WorkStatusRobo);
                if (debug>1) printf("Target:(%lld,%lld)\n", ((T1Map2 *)pMap)->GetPoint2('2',meWorkline2-1).lon, ((T1Map2 *)pMap)->GetPoint2('2',meWorkline2-1).lat);
            } else {
                SetCutGrid();
            }
        }
    }
    //printf("T1ActRoboAll-Work2 end: (%d,%d,%d), WS6=%d, Min:(%lld,%lld), Max:(%lld,%lld)\n", meWorklength, meWorkline, meWorkstep, WorkStatusRobo, mapMin.lon, mapMin.lat, mapMax.lon, mapMax.lat);
    return WorkStatusRobo;
}

/* return==1: working,  return<0: finished work,
 * this include the action from ChargeStation and back to ChargeStation
 */
int T1ActRoboAll::WorkAll() {
    int debug=1;
    int ret;
    static int count=0;
    static MapPoint CutBegin, CutEnd;
    static int flagReverse = 0; // 0 means error
    //printf("T1ActRoboAll-WorkAll begin, myRoboI(%d)\n", myRoboI.yaw);
    CheckSensors();  // check stop sensors first; may set FlagPause=1
    if ( WorkStatusRoboAll < 0 || FlagPause == 1) { // Pause work
        //printf("T1ActRoboAll-WorkAll step0, myRoboI(%d)\n", myRoboI.yaw);
        ;
    } else if ( FlagGoHome == 1 ) { // save the point, and gohome now (triggered by 'h')
        if ( WorkStatusGH < 0 ) WorkStatusRoboAll = 71;
        FlagGoHome = 0;
    } else if ( WorkStatusRoboAll == 0 ) {
        printf("\n\n\n========== T1ActRoboAll-WorkAll, WSRoboAll=0 =============\n");
        RoboAct('I',195);
        pMap->GetRoute(0); pMap->SetAprilTagChrgStation(meFact.StationLength);
        mapBase = pMap->GetPoint('b',0); mapMin = pMap->GetPoint('i',0); mapMax = pMap->GetPoint('a',0);
        //if ( FlagTestNight==8 )
        //    if ( mapBase.lon!= -1 || mapBase.lat!= -1 )
        //        mapBase = myPosRTK;
        strcpy(meStrSendVis,"0"); // range: 0--9, default is 9, 0 is always high resolution
        //FlagStop=0;
        WorkStatusStaOut = 0;
        WorkStatusRoboAll = 1;
        printf("\nT1ActRoboAll-WorkAll step0: start to work all. length=%d, IMU=%d\n", pMap->GetLength('L'), myRoboI.yaw);
    } else if ( WorkStatusRoboAll == 1 ) {
        ret = StationOut();
        // ret = StationOutTest();
        //int retSO = -1;
        if ( ret < 0 ) {
            WorkStatusStaOut = -1;
            //WorkStatusR = 0;
            //WorkStatusRoboAll = 2;
            //if ( FlagTestNight==8 )
                if ( mapBase.lon!= -1 || mapBase.lat!= -1 )
                    mapBase = myPosRTK;
            WorkStatusRoboAll = 11;
            //printf("T1ActRoboAll-WorkAll step1: station out ended.\n");  
        }
    //} else if ( WorkStatusRoboAll == 2 ) { // never do this
        //ret = Work();
        //if ( ret < 0 ) {
            //WorkStatusStaIn = 0;
            //WorkStatusRoboAll = 3;
            //printf("T1ActRoboAll-WorkAll step2: work() ended.\n");
        //}
    } else if ( WorkStatusRoboAll == 3 ) {
        //sprintf(StrCmdSend,"@CMD,M\r\n");
        ret = GoHome();// go home A* path
        if ( ret < 0 ) {
            if ( GoHomeReadPoint() == 1 ) {
                WorkStatusStaIn = 0;
                WorkStatusRoboAll = 5;
                printf("T1ActRoboAll-WorkAll step3->5: return home\n");
            } else {
                WorkStatusGB = 0;
                WorkStatusRoboAll = 4;
                printf("T1ActRoboAll-WorkAll step3->4: return home\n");
            }
            //printf("T1ActRoboAll-WorkAll step3: return home\n");
        }
    } else if ( WorkStatusRoboAll == 4 ) {
        ret = GoBorder();
        if( ret < 0 ) {
            WorkStatusStaIn = 0;
            WorkStatusRoboAll = 5;
            printf("T1ActRoboAll-WorkAll step4: GoBorder ended.\n");
        }
    } else if ( WorkStatusRoboAll == 5 ) { // go in the charge station
        if ( FlagTestNight==8 ) ret=-1;
        else {
            if ( meFact.DriveWheel==0 ) {
                ret = StationInRTK(); // RTK
            } else {
                if (meFact.TestVision == 0){
                    ret = StationInVision(); // Vision 
                } else {
                    ret = StationInVisionXS(); // Vision
                }
            }
        } 
        if( ret < 0 ) {
            WorkStatusStaIn = -1;
            WorkStatusRoboAll = -1;
            WorkStatusR = -1; // ???something the matter
            printf("T1ActRoboAll-WorkAll step5: station in completed\n");
        }
    } else if ( WorkStatusRoboAll == 11 ) { // prepare for the RoutePlan lines
        meWorklength = pMap->GetLength('L'); meWorkline = 0; meWorkstep = 0;
        //myRoboM.fx = 0;  myRoboM.fy = 0;
        flagReverse = 0;
        if ( GoHomeReadPoint() == 1 ) { // have saved point
            WorkStatusRoboAll = 81;
        } else { // no saved point
            meWorkline = 0;
            WorkStatusRoboAll = 12;
        }
        if (debug>1) printf("T1ActRoboAll-WorkAll step11->%d: Judge if GoHome saved. myRoboM(%.0f,%.0f.%d)\n", WorkStatusRoboAll, myRoboM.fx,myRoboM.fy,myRoboM.yaw);
    } else if ( WorkStatusRoboAll == 12 ) { // calculate Start or End point to go
        if (debug>0) printf("\nT1ActRoboAll-WorkAll step12a: prepare to go to the next line's terminal. myRoboM=(%.0f,%.0f,%d)\n", myRoboM.fx,myRoboM.fy,myRoboM.yaw);
        if ( meWorkline >= meWorklength ) {
            printf("T1ActRoboAll-WorkAll step12b, scaned all lines: [%d,meWorkline=(%d,%d)], WSSimu=%d\n", meWorklength, meWorkline, meWorkstep, WorkStatusRobo);
            GoHomeClearPoint();
            WorkStatusGH = 0;
            WorkStatusRoboAll = 3;
        } else {
            // go to the near termial of the next line
            flagReverse = ((T1Map2 *)pMap)->findAstarNear(flagReverse, myRoboM.fx, myRoboM.fy, meWorkline);
            if ( flagReverse == 0 ) { //failed, mark current line as cut and let step 16 find next
                printf("T1ActRoboAll-WorkAll step12b warning, findAstarNear() failed for line %d, marking as cut.\n", meWorkline);
                // Mark current line as cut so findNearestLineBY won't find it again
                MapPoint lineStart = ((T1Map2 *)pMap)->GetPoint2('L', meWorkline);
                MapPoint lineEnd = ((T1Map2 *)pMap)->GetPoint2('l', meWorkline);
                ((T1Map2 *)pMap)->SetGridLineCut((int)lineStart.lon, (int)lineStart.lat, (int)lineEnd.lon, (int)lineEnd.lat);
                if (debug>0) printf("T1ActRoboAll-WorkAll step12b->16, findAstarNear failed, go to step16 to find next line.\n");
                WorkStatusRoboAll = 16; // let step 16 find the next reachable line
            } else {
                if ( flagReverse > 0 ) { // >=1, from Start to End
                    CutBegin = ((T1Map2 *)pMap)->GetPoint2('L',meWorkline);
                    CutEnd = ((T1Map2 *)pMap)->GetPoint2('l',meWorkline);
                } else {  // <=-1, from End to Start
                    CutBegin = ((T1Map2 *)pMap)->GetPoint2('l',meWorkline);
                    CutEnd = ((T1Map2 *)pMap)->GetPoint2('L',meWorkline);
                }
                if (debug>1) printf("T1ActRoboAll-WorkAll step12c, get the RoutePlan2.length=%d\n", pMap->GetLength('2'));
                //int dist = sqrt ( pow(CutBegin.lon-myRoboM.fx,2) + pow(CutBegin.lat-myRoboM.fy,2)  );
                int dist = abs(flagReverse) * ((T1Map2 *)pMap)->GetLength('g');
                if (debug>1) printf("T1ActRoboAll-WorkAll step12d: prepare to workall lines. myRoboM(%.1f,%.1f),targetPoint(%lld,%lld), dist=%d\n",
                    myRoboM.fx, myRoboM.fy, CutBegin.lon, CutBegin.lat, dist);
                int cutter = ((T1Map2 *)pMap)->GetLength('c');
                if ( dist < cutter ) { // too close, no need to go
                    if (debug>0) printf("T1ActRoboAll-WorkAll step12e->15, less than 1 cutter.\n");
                    WorkStatusRoboAll = 15;
                } else if ( dist < cutter*3 ) { // not far, go without Astar
                    if (debug>0) printf("T1ActRoboAll-WorkAll step12e->14, less than 2 cutter.\n");
                    WorkStatusRoboAll = 14;
                } else { // far, go Astar
                    if (debug>0) printf("T1ActRoboAll-WorkAll step12e->13, far than 2 cutter.\n");
                    WorkStatusRoboAll = 13;
                }
                //RoboAct('I',195);
                WorkStatusRobo = 0;
            }
        }
    } else if ( WorkStatusRoboAll == 13 ) {  // walk follow the Astar path
        ret = Work2(); // SetCutGrid() inside the Work2;
        if( ret < 0 ) {
            WorkStatusRoboAll = 15;
            //meWorkline2 = 0;
            printf("T1ActRoboAll-WorkAll step13: work2().%d\n", myDiffMI.yaw);
        }
    } else if ( WorkStatusRoboAll == 14 ) {  // walk directlly to the begin
        //printf("T1ActRoboAll-WorkAll  step14, begin.\n");
        ret = WorkLine(CutBegin);
        if( ret == 0 ) {
            WorkStatusRoboAll = 15;
            printf("T1ActRoboAll-WorkAll  step14a, goto line's begin: revs=%d, line(%d,%d,%d), RSW=%d\n", flagReverse, meWorklength, meWorkline, meWorkstep, WorkStatusRobo);
        } else {
            SetCutGrid();
        }
    } else if ( WorkStatusRoboAll == 15 ) { // walk the RoutePlan' line
        //printf("T1ActRoboAll-WorkAll  step15, begin.\n");
        ret = WorkLine(CutEnd);
        if ( ret == 0 ) {
            SetCutLine(CutBegin,CutEnd);
            meWorkline++;
            if (debug>2) printf("T1ActRoboAll-WorkAll  step15a, cut one line to end: rev=%d, line(%d,%d,%d), RSW=%d\n", flagReverse, meWorklength, meWorkline, meWorkstep, WorkStatusR);
            //printf("TargetS:(%lld,%lld),(%.1f,%.1f)\n", ((T1Map2 *)pMap)->GetPoint2('L',meWorkline-1).lon, ((T1Map2 *)pMap)->GetPoint2('L',meWorkline-1).lat, myMower6.fx, myMower6.fy);
            //printf("TargetE:(%lld,%lld),(%.1f,%.1f)\n", ((T1Map2 *)pMap)->GetPoint2('l',meWorkline-1).lon, ((T1Map2 *)pMap)->GetPoint2('l',meWorkline-1).lat, myMower6.fx, myMower6.fy);
            WorkStatusRoboAll = 16;
        } else if ( ret==1000 ) { // obstacle detected
            printf("T1ActRoboAll-WorkAll step15->100, obastacle detected. ret=%d\n", ret);
            CutEnd.lon = myRoboM.fx; CutEnd.lat = myRoboM.fy;
            SetCutLine(CutBegin,CutEnd);
            SetObst(mySgbmWall);
            WorkStatusRoboAll = 100;
        } else {
            SetCutGrid();
        }
    } else if ( WorkStatusRoboAll == 16 ) { // find the nearest uncut RoutePlan's line
        // if all grid have been cut, the return will large then meWorkLength
        meWorkline = ((T1Map2 *)pMap)->findNearestLineBY((int)myRoboM.fx, (int)myRoboM.fy);
        if (debug>2) printf("T1ActRoboAll-WorkAll step16, findnear=%d,%d\n", meWorkline, meWorklength);
        WorkStatusRoboAll = 12;
    } else if ( WorkStatusRoboAll == 71 ) { // prepare GoHome() - save point before going home
        GoHomeSavePoint();
        WorkStatusGH = 0;
        WorkStatusRoboAll = 3;
        if (debug>2) printf("T1ActRoboAll-WorkAll step71: saved point, going home\n");
    } else if ( WorkStatusRoboAll == 81 ) { // go to the saved point when GoHome() completes
        if (debug>2) printf("T1ActRoboAll-WorkAll step81, read:(%.1f,%.1f,%d,%d)\n", GoHomePoint.fx, GoHomePoint.fy, meWorkline, flagReverse);
        WorkStatusRoboAll = 12; // just goto the last line, not goto the saved point
    } else if ( WorkStatusRoboAll == 100 ) { // re-plan all route lines;
        printf("T1ActRoboAl-WorkAll step100, re-plan the routelines. ret=%d, count=%d\n", ret, count);
        ((T1Map2 *)pMap)->GetRoute3(0);
        count=0;
        WorkStatusRoboAll = 101;
    } else if ( WorkStatusRoboAll == 101 ) { // begin workall with new routePlanLines;
        //pMap->GetRoute(0);
        meWorklength = pMap->GetLength('L'); meWorkline = 0; meWorkstep = 0;
        if ( ++count>5 ) WorkStatusRoboAll = 16;
    } else {
        printf("T1ActRoboAl-WorkAll error status: WS2=%d, WSO=%d, WSI=%d, WSR=%d\n", WorkStatusRoboAll, WorkStatusStaOut, WorkStatusStaIn, WorkStatusR);
    }
    //printf("T1ActRoboAll-WorkAll end: %ld, RSW=%d, %d, %d, Min:(%lld,%lld), Max:(%lld,%lld)\n", meWorklength, WorkStatusR, meWorkline, meWorkstep, mapMin.lon, mapMin.lat, mapMax.lon, mapMax.lat);
    return WorkStatusRoboAll;
}

/* Hub: call all individual sensor check functions
 * 2026.06.05: created by Xs
 * 2026.06.05: refactored as hub, by Xs
 */
void T1ActRoboAll::CheckSensors() {
    CheckSensorsStop();
    CheckSensorsCollision();
    CheckSensorsRain();
    CheckSensorsCharge();
}

/* Check boundary stop sensors (senStop1, senStop2) and trigger pause if hit
 * - Edge-triggered: only acts on 0->1 or 1->0 transition
 * - Never clears FlagPause (let user/'p' key own the clear path)
 * 2026.06.05: created by Xs
 */

void T1ActRoboAll::CheckSensorsStop() {
    int debug = 1;
    static bool triggered = false;
    bool boundaryHit = (mySTAStatus.senStop1 == 0 || mySTAStatus.senStop2 == 0);
    if (boundaryHit){
        if (!triggered) {
            if (debug > 1) printf("T1ActRoboAll-CheckSensorsStop: Boundary triggered, senStop1=%d, senStop2=%d\n",
                mySTAStatus.senStop1, mySTAStatus.senStop2);
            triggered = true;
            if (WorkStatusRoboAll != -1) { //Not in the traversal state
                FlagPause = 1;
            }
            RoboAct('g', 0);
        }
    } else if (!boundaryHit && triggered) {
        if (debug>1) printf("T1ActRoboAll-CheckSensorsStop: Boundary released.\n");
        triggered = false;
    }
}

/* Collision sensors (5x, active-high, gated by !MagnetStop).
 * Status: 0=Normal, 2=Roll/pitch check, 3=Backing up, 4=Re-check hall, 5=Wait P cmd
 * Auto-recovery: check roll/pitch for 1s (100 loops), backup 50cm, re-check hall sensor
 * 2026.06.05: created by Xs
 * 2026.06.08: refined, by Xs
 * 2026.06.11: Modify by Xs
 */
void T1ActRoboAll::CheckSensorsCollision() {
    int debug = 2;
    int ret;
    static int count = 0;
    static int WorkStatusCollision = 0;
    
    if (meFact.MagnetStop != 0) return;
    
    bool collisionHit = (mySTAStatus.senFrontLeft  == 1 ||
                         mySTAStatus.senFrontRight == 1 ||
                         mySTAStatus.senCenter     == 1 ||
                         mySTAStatus.senRearLeft   == 1 ||
                         mySTAStatus.senRearRight  == 1);

    if ( WorkStatusCollision == 0 ) { // normal monitoring
        if (collisionHit) {
            if (debug>0) printf("T1ActRoboAll-CheckSensorsCollision step0: Collision triggered, L=%d R=%d C=%d RL=%d RR=%d\n",
                   mySTAStatus.senFrontLeft, mySTAStatus.senFrontRight,
                   mySTAStatus.senCenter, mySTAStatus.senRearLeft, mySTAStatus.senRearRight);
            // initial response: stop and pause
            if (WorkStatusRoboAll != -1) {
                FlagPause = 1;
                count = 0;
                WorkStatusCollision = 1;
                if (debug>0) printf("T1ActRoboAll-CheckSensorsCollision step0->2: Starting roll/pitch check\n");
            }
            RoboAct('g', 0);
        }
    } else if ( WorkStatusCollision == 1 ) { // check roll/pitch stability for 1s
        bool rollPitchOK = (abs(myRoboI.roll) <= 10 && abs(myRoboI.pitch) <= 10);
        if ( !rollPitchOK ) {
            count = 0;
            if (debug>1) printf("T1ActRoboAll-CheckSensorsCollision step2: roll/pitch out of range (roll=%d, pitch=%d), count reset\n",
                   myRoboI.roll, myRoboI.pitch);
        } else {
            count++;
            if ( count >= 100 ) { // 100 loops ≈ 1s
                if (debug>0) printf("T1ActRoboAll-CheckSensorsCollision step2->3: Roll/pitch stable, count=%d, backing up 50cm\n", count);
                RoboAct('S', 50);
                WorkStatusCollision = 2;
            }
        }
    } else if ( WorkStatusCollision == 2 ) { // waiting for backup to complete
        ret = RoboAct('=', 0);
        if ( ret == 0 ) {
            ;
        } else {
            if (debug>0) printf("T1ActRoboAll-CheckSensorsCollision step3->4: Backup completed, ret=%d, re-checking hall\n", ret);
            WorkStatusCollision = 3;
        }
    } else if ( WorkStatusCollision == 3 ) { // re-check hall sensors
        if ( !collisionHit ) {
            if (debug>0) printf("T1ActRoboAll-CheckSensorsCollision step4->0: Hall cleared, auto-recovery success\n");
            FlagPause = 0;
            WorkStatusCollision = 0;
        } else {
            if (debug>0) printf("T1ActRoboAll-CheckSensorsCollision step4->5: Hall still triggered, waiting for P cmd\n");
            WorkStatusCollision = 4;
        }
    } else if ( WorkStatusCollision == 4 ) { // waiting for P command (FlagPause=0)
        if ( FlagPause == 0 ) {
            if (debug>0) printf("T1ActRoboAll-CheckSensorsCollision step5->0: P cmd received, resuming\n");
            WorkStatusCollision = 0;
        }
    } else {
        printf("T1ActRoboAll-CheckSensorsCollision: ERROR status=%d\n", WorkStatusCollision);
    }
}

/* Rain sensor (active-low). On rain: set FlagGoHome=1, WorkAll transitions to state 3.
 * - Edge log on 1->0; clear triggered on 0->1
 * - Continuous FlagGoHome=1 while senRain==0 (so WorkAll picks it up reliably)
 * 2026.06.08: created by Xs
 * 2026.06.08: Edge-trigger FlagGoHome (Xs)
 */
void T1ActRoboAll::CheckSensorsRain() {
    int debug = 2;
    static bool triggered = false;
    bool raining = (mySTAStatus.senRain == 0);
    if (raining) {
        if (!triggered) {
            if (debug > 1) printf("T1ActRoboAll-CheckSensorsRain: Rain detected, senRain=%d\n", mySTAStatus.senRain);
            triggered = true;
            FlagGoHome = 1;
        }
    } else if (triggered) {
        if (debug > 1) printf("T1ActRoboAll-CheckSensorsRain: Rain stopped.\n");
        triggered = false;
    }
}
/* Charge sensor (active-high). On charge: set FlagInCharge=1, call JudgeCharge() to decide if need to pause.
 * - Edge log on 0->1; clear triggered on 1->0
 * - Continuous FlagInCharge=1 while chargeFlag==1 (so JudgeCharge picks it up reliably)
 * 2026.06.08: created by Xs
 */
void T1ActRoboAll::CheckSensorsCharge() {
    //
    int debug = 2;
    static bool triggered = false;
    bool charging = (mySTAStatus.chargeFlag == 1);
    if (charging) {
        if (!triggered) {
            if (debug > 1) printf("T1ActRoboAll-CheckSensorsCharge: Charging detected, chargeFlag=%d\n", mySTAStatus.chargeFlag);
            triggered = true;
            FlagInCharge = 1;
        }
    }else if (triggered) {
        if (debug > 1) printf("T1ActRoboAll-CheckSensorsCharge: Charging stopped.\n");
        FlagInCharge = 0;
        triggered = false;
    }
}
/* set the grids to cutted value
 * 2026.02.25: created by Leo
 * 2026.05.23: modified by Leo
 */
void T1ActRoboAll::SetCutGrid() {
    //printf("T1ActRoboAll-SetCutGrid, [%d,%d,%d], myRoboM=(%.1f,%.1f,%d)\n", meWorkline, meWorkline2, meWorkstep, myRoboM.fx, myRoboM.fy, myRoboM.yaw);
    int width =  ((T1Map2 *)pMap)->GetLength('c');
    int dx = width * cos(3.14159265*myRoboM.yaw/180) / 2;
    int dy = width * sin(3.14159265*myRoboM.yaw/180) * -1 / 2;
    Cleft.fx = myRoboM.fx - dx;
    Cleft.fy = myRoboM.fy - dy;
    Cright.fx = myRoboM.fx + dx;
    Cright.fy = myRoboM.fy + dy;
    //printf("T1ActRoboAll-SetCutGrid, step 1: Cleft(%.1f,%.1f), Cright(%.1f,%.1f)\n", Cleft.fx, Cleft.fy, Cright.fx, Cright.fy);
    ((T1Map2 *)pMap)->SetGridLineCut((int)Cleft.fx, (int)Cleft.fy, (int)Cright.fx, (int)Cright.fy);
    //((T1Map2 *)pMap)->Save2BitMapToFile("../Map/twobitmap.bin");
    //printf("T1ActRoboAll-SetCutGrid, step 2, end\n");
}

/* set the line to cutted value
 * 2026.03.09: modified by Leo
 */
void T1ActRoboAll::SetCutLine(MapPoint begin, MapPoint end) {
    int distx = end.lon-begin.lon;
    int disty = end.lat-begin.lat;
    int dist = sqrt( pow(distx,2) + pow(disty,2) );
    float anglesin = distx / dist;
    float anglecos = disty / dist;
    int width =  ((T1Map2 *)pMap)->GetLength('c');
    int dx = width * anglecos / 2;
    int dy = width * anglesin * -1 / 2;
    int step =  ((T1Map2 *)pMap)->GetLength('g');
    int count = dist / step;
    //printf("T1ActRoboAll-SetCutLine begin(%lld,%lld) end(%lld,%lld), step=（%d,%d)\n", begin.lon,begin.lat,end.lon,end.lat, step, count);
    float stepx = step * anglesin;
    float stepy = step * anglecos;
    long long x = begin.lon;
    long long y = begin.lat;
    for ( int i=0 ; i<=count; i++ ) {
        Cleft.fx = x - dx;
        Cleft.fy = y - dy;
        Cright.fx = x + dx;
        Cright.fy = y + dy;
        //printf("T1ActRoboAll-SetCutLine, step 1: Cleft(%.1f,%.1f), Cright(%.1f,%.1f)\n", Cleft.fx, Cleft.fy, Cright.fx, Cright.fy);
        ((T1Map2 *)pMap)->SetGridLineCut((int)Cleft.fx, (int)Cleft.fy, (int)Cright.fx, (int)Cright.fy);    
        //printf("T1ActRoboAll-SetCutLine loop(%d,%d) point(%lld,%lld)\n", count, i, x, y);
        x += stepx;
        y += stepy;
    }
}

// set the grid to cutted value
void T1ActRoboAll::SetCutGridXs() {
    // Render the "cut" grids along the motion path, filling in-between even if position jumps.
    // This is for visualization: it interpolates from last pose to current pose and draws many
    // small cutter segments so gaps are not left on the grid map.
    static bool hasLast = false;
    static double lastX = 0.0, lastY = 0.0;
    static int lastYaw = 0;
    // Reset state if necessary
    if (WorkStatusRoboAll < 0 ||
        (meWorkline == 0 && meWorkstep <= 1 &&
         WorkStatusRoboAll >= 11 && WorkStatusRoboAll <= 13)) {
        hasLast = false;
    }
    // Current cutter half-width vector (double precision)
    // Use GetStatus('c') to get cutter size from map file if available, otherwise from factory file
    int cutterSize = GetStatus('c');
    const double curYawRad = 3.14159265 * myRoboM.yaw / 180.0;
    const double dx = cutterSize * cos(curYawRad) / 2.0;
    const double dy = cutterSize * sin(curYawRad) * -1.0 / 2.0;
    if (!hasLast) {
        // First time: just draw current cutter width
        ((T1Map2 *)pMap)->SetGridLineCut((int)lround(myRoboM.fx - dx), (int)lround(myRoboM.fy - dy),
                                         (int)lround(myRoboM.fx + dx), (int)lround(myRoboM.fy + dy));
    } else {
        // If we have a last pose, interpolate and fill between last -> current
        const double dX = myRoboM.fx - lastX;
        const double dY = myRoboM.fy - lastY;
        const double dist = sqrt(dX * dX + dY * dY);

        // Always draw the centerline between poses (Bresenham in grid space) to guarantee continuity.
        // This specifically fixes "jump" poses like 10 -> 20 -> 60 leaving empty grids between.
        ((T1Map2 *)pMap)->SetGridLineCut((int)lround(lastX), (int)lround(lastY),
                                         (int)lround(myRoboM.fx), (int)lround(myRoboM.fy));

        // Sampling step: dense enough to avoid gaps. Cap samples to avoid heavy CPU.
        // Use GetStatus('g') to get grid size from map file if available, otherwise from factory file
        int gridSize = GetStatus('g');
        double minSampleDist = gridSize / 4.0;        // quarter grid
        const double cutterSample = cutterSize / 2.0;   // half cutter width
        if (cutterSample < minSampleDist) minSampleDist = cutterSample;
        if (minSampleDist < 1.0) minSampleDist = 1.0;        // safety

        int numSamples = (int)(dist / minSampleDist) + 1;
        if (numSamples < 1) numSamples = 1;
        if (numSamples > 200) numSamples = 200; // avoid too many draws on very large jumps

        // Interpolate yaw (shortest path) to keep cutter direction reasonable
        int yawDiff = myRoboM.yaw - lastYaw;
        if (yawDiff > 180) yawDiff -= 360;
        else if (yawDiff < -180) yawDiff += 360;

        for (int i = 0; i <= numSamples; i++) {
            const double t = (double)i / (double)numSamples;
            const double x = lastX + t * dX;
            const double y = lastY + t * dY;

            int yaw = lastYaw + (int)(t * yawDiff);
            if (yaw < 0) yaw += 360;
            if (yaw >= 360) yaw -= 360;

            const double yawRad = 3.14159265 * yaw / 180.0;
            const double sdx = cutterSize * cos(yawRad) / 2.0;
            const double sdy = cutterSize * sin(yawRad) * -1.0 / 2.0;

            ((T1Map2 *)pMap)->SetGridLineCut((int)lround(x - sdx), (int)lround(y - sdy),
                                             (int)lround(x + sdx), (int)lround(y + sdy));
        }

        // Also draw the edges between last cutter endpoints and current cutter endpoints
        const double lastYawRad = 3.14159265 * lastYaw / 180.0;
        const double lastDx = cutterSize * cos(lastYawRad) / 2.0;
        const double lastDy = cutterSize * sin(lastYawRad) * -1.0 / 2.0;
        ((T1Map2 *)pMap)->SetGridLineCut((int)lround(lastX - lastDx), (int)lround(lastY - lastDy),
                                         (int)lround(myRoboM.fx - dx), (int)lround(myRoboM.fy - dy));
        ((T1Map2 *)pMap)->SetGridLineCut((int)lround(lastX + lastDx), (int)lround(lastY + lastDy),
                                         (int)lround(myRoboM.fx + dx), (int)lround(myRoboM.fy + dy));
    }

    // Remember for next call
    lastX = myRoboM.fx;
    lastY = myRoboM.fy;
    lastYaw = myRoboM.yaw;
    hasLast = true;
}

/* set the grid to wall obstacle based on depth camera data
 * 2026.05.22: created by BY
 */
void T1ActRoboAll::SetObst(Wall wall) {
    printf("T1ActRoboAll-SetObst begin, wall(leftDepth=%.1f,leftOffset=%.1f,rightDepth=%.1f,rightOffset=%.1f), myRoboM(%.1f,%.1f,%d)\n",
        wall.leftDepth, wall.leftOffset, wall.rightDepth, wall.rightOffset,
        myRoboM.fx, myRoboM.fy, myRoboM.yaw);
    float rad = myRoboM.yaw * 0.0174532925f;
    float vcos = cos(rad);
    float vsin = sin(rad);
    float leftDepth = wall.leftDepth;
    float rightDepth = wall.rightDepth;
    // Loop 14 times, decreasing depth by 3.5 each time
    for (int i = 0; i < 12; i++, leftDepth -= 3.5f, rightDepth -= 3.5f) {
        // Exit if either depth becomes negative
        if (leftDepth < 0 || rightDepth < 0) {
            printf("T1ActRoboAll-SetObst exit early, loop(%d), leftDepth=%.1f, rightDepth=%.1f\n", i, leftDepth, rightDepth);
            break;
        }
        PointRobo pleft, pright;
        // left edge: depth along robot forward, offset along lateral (camera X axis)
        pleft.fx = myRoboM.fx + leftDepth * vsin + wall.leftOffset * vcos;
        pleft.fy = myRoboM.fy + leftDepth * vcos - wall.leftOffset * vsin;
        // right edge
        pright.fx = myRoboM.fx + rightDepth * vsin + wall.rightOffset * vcos;
        pright.fy = myRoboM.fy + rightDepth * vcos - wall.rightOffset * vsin;
        ((T1Map2 *)pMap)->SetGridObstacle(pleft.fx, pleft.fy, pright.fx, pright.fy);
        printf("T1ActRoboAll-SetObst loop(%d), leftDepth=%.1f, rightDepth=%.1f, pleft(%.1f,%.1f), pright(%.1f,%.1f)\n",
            i, leftDepth, rightDepth, pleft.fx, pleft.fy, pright.fx, pright.fy);
    }
    printf("T1ActRoboAll-SetObst end\n");
}
