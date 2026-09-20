#include "T1Draw.hpp"
#include "T1Utils.hpp"

#include <iostream>
#include <unistd.h>
#include <math.h>
#include <cstring>
//#include <X11/Xutil.h>


T1Draw::T1Draw() {
    myAxis.init("./upt/axis.upt");
    myAxis.noShift = 1;
    myAxis.IDdevice = 0;
    myAxis.devtype = 0;  // axis
    printf("T1Draw-T1Draw, MoveSet Axis: %d, %d\n", lawnwidth, lawnheight);
    //myAxis.MoveSet( -1*lawnwidth/2, lawnheight/2, 10);    
    //myAxis.MoveSet( -1*lawnwidth/2, 0, -90);
    //Init();
}

T1Draw::~T1Draw() {
    // clean all source
    XFreeGC(myDisplay, myGc);
    XDestroyWindow(myDisplay, myWindow);
    XCloseDisplay(myDisplay);
    std::cout << "T1Draw-~T1Draw(): free all display and windows" << std::endl;

}

bool T1Draw::Init(void) {
    //SetScale(0.5,0.5,-0.5);
    SetRotate(0,30,0);
    myAxis.MoveSet( -1*lawnwidth/2, lawnheight/2, -90);

    // open the X's display
    myDisplay = XOpenDisplay(NULL);
    if (!myDisplay) {
        fprintf(stderr, "can not open the myDisaplyʾ\n");
        return false;
    }

    // get the default screen
    myScreen = DefaultScreen(myDisplay);
    
    //XSizeHints hints;
    //hints.flags = USPosition;
    //hints.x = 10;
    //hints.y = 10; 
    //XSetWMNormalHints(myDisplay, myWindow, &hints);
    // creat the window
    myWindow = XCreateSimpleWindow(
        myDisplay, RootWindow(myDisplay, myScreen),
        200, 200, winwidth, winheight, 1,
//        100, 100, 1600, 900, 1,
        BlackPixel(myDisplay, myScreen),
        WhitePixel(myDisplay, myScreen)
    );
       
    if (myWindow == 0) {
        fprintf(stderr, "failed to creat myWindow\n");
        XCloseDisplay(myDisplay);
        return false;
    }

    // set the window's title
    XStoreName(myDisplay, myWindow, "Lawn Mower Mapping");
    
    // select the event need access
    XSelectInput(myDisplay, myWindow, ExposureMask | KeyPressMask);
    
    // flush the windows
    XMapWindow(myDisplay, myWindow);
    XMoveWindow(myDisplay, myWindow, 600, 360);
    XFlush(myDisplay);
    
    // set the graphics context
    myGc = XCreateGC(myDisplay, myWindow, 0, NULL);
    XSetForeground(myDisplay, myGc, BlackPixel(myDisplay, myScreen));
    
    std::cout << "T1Draw-Init(): seccess." << std::endl;
    return true;
}

// process event and flush the display
void T1Draw::myFlush() {
    while (XPending(myDisplay)) {
        XEvent ev;
        XNextEvent(myDisplay, &ev);
        if (ev.type == KeyPress) {
            KeySym key = XLookupKeysym(&ev.xkey, 0);
            switch (key) {
                case XK_g: SignalKey='g'; break;  // stop
                case XK_q: SignalKey='q'; break;  // qwe
                case XK_w: SignalKey='w'; break;  // qwe
                case XK_f: SignalKey='f'; break;   
                case XK_e: SignalKey='e'; break;  // qwe
                case XK_a: SignalKey='a'; break;  // asd
                case XK_s: SignalKey='s'; break;  // asd
                case XK_d: SignalKey='d'; break;  // asd
                case XK_z: SignalKey='z'; break;  // zxc
                case XK_x: SignalKey='x'; break;  // zxc
                case XK_c: SignalKey='c'; break;  // zxc
                case XK_r: SignalKey='r'; break;  // real workAll, same as 3
                case XK_t: SignalKey='t'; break;  // trail RTK point to log map, same as 1
                case XK_p: SignalKey='p'; break;  // pause|play
                case XK_comma: SignalKey='<'; break;  // select map ++
                case XK_period: SignalKey='>'; break;  // select map --
                case XK_minus: SignalKey='-'; break;  // delete map
                case XK_i: SignalKey='i'; break;  // input command
                case XK_j: SignalKey='j'; break;  // fake vision 50
                case XK_k: SignalKey='k'; break;  // fake vision 60
                case XK_h: SignalKey='h'; break;  //return home
                case XK_l: SignalKey='l'; break;  // line type of route
                case XK_b: SignalKey='b'; break;  // sgbm
                case XK_o: SignalKey='o'; break;  // exit
                case XK_u: SignalKey='u'; break;  // update
                case XK_1: SignalKey='1'; break;  // log RTK trails when user remoto walk on lawn
                case XK_2: SignalKey='2'; break;  // real workAll include station Out/In
                case XK_3: SignalKey='3'; break;  // real workAll with Astar
                case XK_4: SignalKey='4'; break;  // real work, goto target by X3
                //case XK_5: SignalKey='5'; break;  // SMD real work by 415 to target
                case XK_6: SignalKey='6'; break;  // simulate move on screen
                case XK_7: SignalKey='7'; break;  // simulate All route on screen
                case XK_8: SignalKey='8'; break;  // fake vision obstacle 50cm ahead
                //case XK_9: SignalKey='9'; break;  // convert RTK Log to map
                case XK_0: SignalKey='0'; break;  // stop real and simulate work
                default: SignalKey=0; break;
            }
            //std::cout << "key pressed and exit" << std::endl;
            // can stop loop by special key here
        } else if (ev.type == Expose) {
            // redraw windows
            redrawCurrentFrame();
        }
    }
}

// redraw the frame
void T1Draw::redrawCurrentFrame() {
    // redraw

    XFlush(myDisplay);
}

void T1Draw::DrawCommon(const char * name, const char *verX3 , const char * verF415) {
    //std::cout << "T1Draw-DrawCommon(): " << name << std::endl;
    if (!myDisplay || !myWindow || !myGc) {
        std::cerr << "T1Draw-test2(): the  context are not ready" << myDisplay << "," << myWindow << "," << myGc << "," << myScreen << std::endl;
        return;
    }
    
    // clear the display
    XSetForeground(myDisplay, myGc, 0xF0F0F0);
    XFillRectangle(myDisplay, myWindow, myGc, 0, 0, winwidth, winheight);
    XSetForeground(myDisplay, myGc, 0xFFFFFF);
    XFillRectangle(myDisplay, myWindow, myGc, 0, 0, winwidth/4, winheight/4.8);
    // actions: display time
    std::string text = name;
    //text +=   "count: ";
    XSetForeground(myDisplay, myGc, 0x0000FF);
    XDrawString(myDisplay, myWindow, myGc, 5, 12, text.c_str(), text.length());
    text = std::string("VerX3: ") + verX3;
    //text = verX3;
    XDrawString(myDisplay, myWindow, myGc, 5, 25, text.c_str(), text.length());
    text = std::string("VerF415: ") + verF415;
    XDrawString(myDisplay, myWindow, myGc, 100, 25, text.c_str(), text.length());
    text = std::string("SN: ") + T1Utils::meSsid;
    XDrawString(myDisplay, myWindow, myGc, 220, 25, text.c_str(), text.length());
    //XFlush(myDisplay);
    
    // draw axis
    //DrawMower(&myAxis);
    int x1=winwidth/5, y1=winheight/9,x2,y2;
    x2 = x1 + 50;   y2 = y1;
    XDrawLine(myDisplay, myWindow, myGc, x1,y1,x2,y2);
    XDrawLine(myDisplay, myWindow, myGc, x2-10,y2-5,x2,y2);
    XDrawLine(myDisplay, myWindow, myGc, x2-10,y2+5,x2,y2);
    x2 = x1; y2 = y1 - 50;
    XDrawLine(myDisplay, myWindow, myGc, x1,y1,x2,y2);
    XDrawLine(myDisplay, myWindow, myGc, x2-5,y2+10,x2,y2);
    XDrawLine(myDisplay, myWindow, myGc, x2+5,y2+10,x2,y2);
    x2 = x1 + 30 * rotation.dxsin; //DrawGetRotaX( x1+50, y1-50, 0);
    y2 = y1 - 30 * rotation.dycos; //DrawGetRotaY( y1-50, 0, 0);
    XDrawLine(myDisplay, myWindow, myGc, x1,y1,x2,y2);
    XDrawString(myDisplay, myWindow, myGc, x2-5, y2-5, "XYZ=ENU", 7);

    XSetForeground(myDisplay, myGc, 0xAFAFFF);
    // draw lawn cross
    x1 = DrawGetRotaX(-1*lawnwidth/2, 0, 0);
    y1 = DrawGetRotaY(0, 0, 0);
    x2 = DrawGetRotaX(lawnwidth/2, 0, 0);
    y2 = DrawGetRotaY(0, 0, 0);    
    XDrawLine(myDisplay, myWindow, myGc, x1,y1,x2,y2);
    for ( int i=1000; i<=lawnwidth/2; i+=1000 ) { // draw X axis unit, 10M
        x1 = DrawGetRotaX( i,100, 0);
        y1 = DrawGetRotaY(100, 0, 0);
        
        x2 = DrawGetRotaX( i, -100, 0);
        y2 = DrawGetRotaY(-100, 0, 0);
        //printf("T1Draw-DrawCommon, aixs unit: %d, %d, %d, %d, %d\n", i, x1, y1, x2, y2);
        XDrawLine(myDisplay, myWindow, myGc, x1, y1, x2, y2);
        x1 = DrawGetRotaX( -1*i,100, 0);
        x2 = DrawGetRotaX( -1*i, -100, 0);
        //printf("T1Draw-DrawCommon, aixs unit: %d, %d, %d, %d, %d\n", i, x1, y1, x2, y2);
        XDrawLine(myDisplay, myWindow, myGc, x1, y1, x2, y2);
    }
    x1 = DrawGetRotaX(0, -1*lawnheight/2, 0);
    y1 = DrawGetRotaY(-1*lawnheight/2, 0, 0);
    x2 = DrawGetRotaX(0, lawnheight/2, 0);
    y2 = DrawGetRotaY(lawnheight/2, 0, 0);    
    //XSetForeground(myDisplay, myGc, 0xFF80FF);
    XDrawLine(myDisplay, myWindow, myGc, x1,y1,x2,y2);
    for ( int i=1000; i<=lawnheight/2; i+=1000 ) { // draw Y axis unit, 10M
        x1 = DrawGetRotaX(-100,i, 0);
        y1 = DrawGetRotaY(i, 0, 0);
        x2 = DrawGetRotaX(100,i, 0);
        y2 = DrawGetRotaY(i, 0, 0);
        //printf("T1Draw-DrawCommon, aixs unit: %d, %d, %d, %d, %d\n", i, x1, y1, x2, y2);
        XDrawLine(myDisplay, myWindow, myGc, x1, y1, x2, y2);
        x1 = DrawGetRotaX(-100, -1*i, 0);
        y1 = DrawGetRotaY(-1*i, 0, 0);
        x2 = DrawGetRotaX(100, -1*i, 0);
        y2 = DrawGetRotaY(-1*i, 0, 0);
        //printf("T1Draw-DrawCommon, aixs unit: %d, %d, %d, %d, %d\n", i, x1, y1, x2, y2);
        XDrawLine(myDisplay, myWindow, myGc, x1, y1, x2, y2);
    }
    //test2();
    //test();
    // process event and flush the display
    //myFlush();
}

/************ test X11 windwos ************/
//int main() {
//    test();
//}

void T1Draw::rotate_point(int cx, int cy, int x, int y, float angle, int* rx, int* ry) {
    float rad = angle * M_PI / 180.0f;
    *rx = cx + (int)((x - cx) * cos(rad) - (y - cy) * sin(rad));
    *ry = cy + (int)((x - cx) * sin(rad) + (y - cy) * cos(rad));
}

// draw one frame
void T1Draw::test2() {

    for ( int i=0; i<10; i ++ ) {
        // draw trotate red line
        int lineX1, lineY1, lineX2, lineY2;
        rotate_point(lineCenterX, lineCenterY, 
                    lineCenterX - lineLength/2, lineCenterY, 
                    lineAngle, &lineX1, &lineY1);
        rotate_point(lineCenterX, lineCenterY, 
                    lineCenterX + lineLength/2, lineCenterY, 
                    lineAngle, &lineX2, &lineY2);
        XSetForeground(myDisplay, myGc, 0xFF0000);
        XDrawLine(myDisplay, myWindow, myGc, lineX1, lineY1, lineX2, lineY2);
            //XSetForeground(myDisplay, myGc, 0xFF8600);
            //XDrawLine(myDisplay, myWindow, myGc, 10, 10, 200, 200);
        
        // draw moving green circle
        XSetForeground(myDisplay, myGc, 0x00FF00);
        XFillArc(myDisplay, myWindow, myGc, 
                circleX - radius, circleY - radius, 
                radius * 2, radius * 2, 
                0, 360*64);
        
        // change moving argument
        circleX += moveStep;
        if ( circleX > 800 ) circleX = 300;
        lineAngle += moveStep;
        //printf("draw: cirlceX=%d, lineAngle=%f\n",circleX,lineAngle);
    }

}

/************ test X11 windwos end ************/
    
    
void T1Draw::DrawMower(T1Mower *mower) {
    if ( mower->devtype == 0 ) { // axis
        XSetForeground(myDisplay, myGc, 0x0000FF);//painter->setPen(QColor(4,228,255));
    } else if ( mower->devtype == 101 ) { // SANF01 mower
        XSetForeground(myDisplay, myGc, 0x2020FF);//0xFFC600);//painter->setPen(QColor(225,198,0));
    } else {
        XSetForeground(myDisplay, myGc, 0x0F0F0F); // black//painter->setPen(Qt::white);
        printf("T1Draw-DrawMower, unknown type. %d\n", mower->devtype);
    }
    int x1,y1,x2,y2,x3,y3,x4,y4;
    MyPoint prb,pre;
    // draw lines
    for ( int i=0; i<mower->body.size(); i++ ) {
        if ( mower->body[i].radius > 0 ) { // the test line
            continue;
        } else if ( mower->body[i].radius < 0 ) { // front/top/side surface
            continue;
        } else { // 0, the display lines
            prb = mower->RotateAxis(mower->body[i].type,mower->body[i].begin);
            pre = mower->RotateAxis(mower->body[i].type,mower->body[i].end);
            x1 =  DrawGetRotaX(prb.x+mower->move.x, prb.y+mower->move.y, mower->noShift);
            y1 =  DrawGetRotaY(prb.y+mower->move.y, prb.z+mower->move.z, mower->noShift);
            x2 =  DrawGetRotaX(pre.x+mower->move.x, pre.y+mower->move.y, mower->noShift);
            y2 =  DrawGetRotaY(pre.y+mower->move.y, pre.z+mower->move.z, mower->noShift);
            //painter->drawLine(x1,y1,x2,y2);
            //printf("T1Draw-DrawMower, display line: %d,%d,%d,%d,%d,%d,%d\n",i,prb.x,prb.y,x1,y1,x2,y2);
            XDrawLine(myDisplay, myWindow, myGc, x1, y1, x2, y2);
            //XDrawLine(myDisplay, myWindow, myGc, 10, 10, 300, 200);
        }
    }
    
    // draw lable
    char tmpstr[30]="(XYZ==ENU)"; // axis
    if ( mower->IDdevice == 101 )
        strcpy(tmpstr, "101");

    if ( mower->devtype == 0 ) { // axis
        XDrawString(myDisplay, myWindow, myGc, x1-5, y1+20, tmpstr, strlen(tmpstr));
    } else if ( mower->devtype == 101 ) { // mower
        XDrawString(myDisplay, myWindow, myGc, x1, y1+15, tmpstr, strlen(tmpstr));
    } else {
        ;
    }

    /* draw signal lamp
    if ( mower->IDdevice == 101 )
        ;//painter->drawText(rectangle3, 0, tmpstr3, &boundingRect);
    // qDebug() << "T1Draw-drawmower draw RTK1-2.";
    int ret;
    ret = mower->mowerFixedP;
    if ( ret == 4 )
        painter->setPen(Qt::cyan );
    else if ( ret == 5 )
        painter->setPen(Qt::yellow);
    else if ( ret == 2 )
        painter->setPen(Qt::darkYellow);
    else if ( ret == 1 )
        painter->setPen(Qt::darkGreen);
    else
        painter->setPen(Qt::darkRed);
    if ( mower->noShift == 0 ) {  // not the axis
        prb = mower->RotateAxis(1,mower->fixRTK1);  // fixRTK1 is type 1
        x1 =  DrawGetRotaX(prb.x+mower->move.x, prb.y+mower->move.y,0);
        y1 =  DrawGetRotaY(prb.y+mower->move.y, prb.z+mower->move.z,0);
        painter->drawRect(x1-2,y1-5,4,10);
    }
    ret = mower->mowerFixedA;
    if ( ret == 4 )
        painter->setPen(Qt::cyan );
    else if ( ret == 5 )
        painter->setPen(Qt::yellow);
    else if ( ret == 2 )
        painter->setPen(Qt::darkYellow);
    else
        painter->setPen(Qt::darkGreen);
    if ( mower->IDRtk2 != 0 ) {  // 300T has only 1 RTK
        prb = mower->RotateAxis(2,mower->fixRTK2);  // fixRTK2 is type 2
        x2 =  DrawGetRotaX(prb.x+mower->move.x, prb.y+mower->move.y,0);
        y2 =  DrawGetRotaY(prb.y+mower->move.y, prb.z+mower->move.z,0);
        painter->drawRect(x2-2,y2-5,4,6);
    }
    //DrawLidar(painter,mower);
    */
    //myFlush();
}

// draw the cutted line of a mower
void T1Draw::DrawCutLine(PointRobo Cl, PointRobo Cr) {
    int x1,y1,x2,y2,x3,y3,x4,y4;

    XSetForeground(myDisplay, myGc, 0xA00000);
    x1 =  DrawGetRotaX(Cl.fx, Cl.fy, 0);
    y1 =  DrawGetRotaY(Cl.fy, 0, 0);
    x2 =  DrawGetRotaX(Cr.fx, Cr.fy, 0);
    y2 =  DrawGetRotaY(Cr.fy, 0, 0);
    //printf("T1Draw-DrawMower, display line: %d,%d,%d,%d,%d,%d,%d\n",i,prb.x,prb.y,x1,y1,x2,y2);
    XDrawLine(myDisplay, myWindow, myGc, x1, y1, x2, y2);

}

// draw the (x,y,yaw) of mRoboM from at32-f415
void T1Draw::DrawActStatus(T1Act *act) {
    MapPoint tmpPoint;
    char tmpstr[100];
    char WS='x';

    XSetForeground(myDisplay, myGc, 0x007F00);

    tmpPoint = act->GetPoint('r',0);
    sprintf(tmpstr, "Base_415:(%lld,%lld)   count: %d", tmpPoint.lon, tmpPoint.lat, act->GetStatus('C'));
    XDrawString(myDisplay, myWindow, myGc, 15, 85, tmpstr, strlen(tmpstr));
    if ( act->GetStatus('T') != -1 ) WS='T';
    if ( act->GetStatus('S') != -1 ) WS='S';
    if ( act->GetStatus('5') != -1 ) WS='5';
    if ( act->GetStatus('R') != -1 ) WS='R';
    tmpPoint = act->GetPoint('M',0);
    sprintf(tmpstr, "RobotMem:(%lld,%lld,%d) WS:%c", tmpPoint.lon, tmpPoint.lat, tmpPoint.yaw, WS);
    XDrawString(myDisplay, myWindow, myGc, 15, 100, tmpstr, strlen(tmpstr));
    tmpPoint = act->GetPoint('I',0);
    if ( act->GetStatus('I') && 0x01 ) {
        sprintf(tmpstr, "RobotIMU:(%d,%d,%d)  AgS:%d,%d  AgN-%d:%d,%d", tmpPoint.roll, tmpPoint.pitch, tmpPoint.yaw, act->GetStatus('s'), act->GetStatus('['), act->GetStatus('e'), act->GetStatus('n'), act->GetStatus(']')  );
        XDrawString(myDisplay, myWindow, myGc, 15, 115, tmpstr, strlen(tmpstr));
    } else {
        XSetForeground(myDisplay, myGc, 0xFF0000);
        sprintf(tmpstr, "RobotIMU:( receive error! )  AgS:%d,  AgN-%d:%d", act->GetStatus('s'), act->GetStatus('e'), act->GetStatus('n') );
        XDrawString(myDisplay, myWindow, myGc, 15, 115, tmpstr, strlen(tmpstr));
        XSetForeground(myDisplay, myGc, 0x007F00);
    }
    tmpPoint = act->GetPoint('K',0);
    if ( act->GetStatus('P') == -1 ) {
        XSetForeground(myDisplay, myGc, 0xFF0000);
        sprintf(tmpstr, "RobotRTK:( receive error! )  Fix:%d  SataA:%d", act->GetStatus('P'), act->GetStatus('S'));
        XDrawString(myDisplay, myWindow, myGc, 15, 130, tmpstr, strlen(tmpstr));
        XSetForeground(myDisplay, myGc, 0x007F00);
    } else {
        sprintf(tmpstr, "RobotRTK:(%lld,%lld)  Fix:%d  SataA:%d", tmpPoint.lon, tmpPoint.lat, act->GetStatus('P'), act->GetStatus('S'));
        XDrawString(myDisplay, myWindow, myGc, 15, 130, tmpstr, strlen(tmpstr));
    }
    tmpPoint = act->GetPoint('F',0);
    sprintf(tmpstr, "RobotFix:(%lld,%lld)", tmpPoint.lon, tmpPoint.lat);
    XDrawString(myDisplay, myWindow, myGc, 15, 145, tmpstr, strlen(tmpstr));
    
    tmpPoint = act->GetPoint('V',0);
    // display battery level
    double voltage = tmpPoint.batteryVoltage;
    if ( voltage > 0.0 ) {
        constexpr double fullVoltage = 21.0;   // 5S 18650 full voltage
        constexpr double emptyVoltage = 15.75; 
        double ratio = (voltage - emptyVoltage) / (fullVoltage - emptyVoltage);
        int batteryLevel = static_cast<int>(ratio * 100.0);
        
        // set color based on battery level
        if ( batteryLevel < 20 ) {
            XSetForeground(myDisplay, myGc, 0x7F0000);  // red when need recharge
        } else if ( batteryLevel < 50 ) {
            XSetForeground(myDisplay, myGc, 0x9F3F00);  // orange when low
        } else if ( batteryLevel < 80 ) {
            XSetForeground(myDisplay, myGc, 0x9F9F00);  // yellow when medium
        } else {
            XSetForeground(myDisplay, myGc, 0x009F00);  // green when high
        }
        
        if (batteryLevel < 0) batteryLevel = 0;
        if (batteryLevel > 100) batteryLevel = 100;
        //act->batteryLevel = batteryLevel;
        sprintf(tmpstr, "Battery: %d%% (%.2fV)", batteryLevel, voltage);
        XDrawString(myDisplay, myWindow, myGc, 15, 160, tmpstr, strlen(tmpstr));
        XSetForeground(myDisplay, myGc, 0x003F00);  // restore color
    } else {
        XSetForeground(myDisplay, myGc, 0xFF0000);
        sprintf(tmpstr, "Battery: ( no data )");
        XDrawString(myDisplay, myWindow, myGc, 15, 160, tmpstr, strlen(tmpstr));
        XSetForeground(myDisplay, myGc, 0x007F00);
    }
    
    //myFlush();
}

void T1Draw::DrawGrid(int x, int y, int gridsize, int color, int minx, int miny) {

    int tmpx,tmpy,x1,y1,x2,y2,x3,y3,x4,y4,fullcolor;//lenx,leny;
//printf("T1Draw-DrawGrid begin, (%d,%d,%d,%d)\n", x, y, len, color);
    //if ( color == 0 ) return;
    x--; y--;
    //tmpx = (x-0.4)*gridsize + minx; tmpy = (y-0.4)*gridsize + miny;
    tmpx = (x+0.2)*gridsize + minx; tmpy = (y+0.2)*gridsize + miny;
    x1=DrawGetRotaX(tmpx, tmpy, 0);
    y1=DrawGetRotaY(tmpy, 0, 0);
    //tmpx = (x+0.4)*gridsize + minx; tmpy = (y-0.4)*gridsize + miny;
    tmpx = (x+0.8)*gridsize + minx; tmpy = (y+0.2)*gridsize + miny;
    x2=DrawGetRotaX(tmpx, tmpy, 0);
    y2=DrawGetRotaY(tmpy, 0, 0);
    //tmpx = (x+0.4)*gridsize + minx; tmpy = (y+0.4)*gridsize + miny;
    tmpx = (x+0.8)*gridsize + minx; tmpy = (y+0.8)*gridsize + miny;
    x3=DrawGetRotaX(tmpx, tmpy, 0);
    y3=DrawGetRotaY(tmpy, 0, 0);
    //tmpx = (x-0.4)*gridsize + minx; tmpy = (y+0.4)*gridsize + miny;
    tmpx = (x+0.2)*gridsize + minx; tmpy = (y+0.8)*gridsize + miny;
    x4=DrawGetRotaX(tmpx, tmpy, 0);
    y4=DrawGetRotaY(tmpy, 0, 0);

    //lenx = len/2 * rotation.scale_x;
    //leny = len/2 * rotation.scale_y;
    //color =  255 - color;
    //fullcolor =  color*65536 + color*256 + color;
    if ( color == VALUEGFORB ) fullcolor=0x7F7F7F;   // forbidden area
    else if ( color == VALUEGBORD ) fullcolor=0xA0E0A0;    // border
    //else if ( color == 20 ) fullcolor=0xA0A0E0;    // Astar path
    else if ( color == VALUEGASTA ) fullcolor=0xD0D000;    // Astar path
    else if ( color == VALUEGGRASS ) fullcolor=0xFFFFFF;    // grass unit
    else if ( color == VALUEGPASSONLY ) fullcolor=0x7F7F00;    // only passable unit
    else if ( color == VALUEGCUT ) fullcolor=0xF0B0D0;    // grid has been cut
    else if ( color == VALUEGTAG ) fullcolor=0x0000FF;    // AprilTag areaf
    else if ( color > 0 ) fullcolor=0x70D0D0;    // Line Grid path 
    else if ( color <= VALUEGORBSTACLE ) fullcolor=0x7F007F;    // Obstacle Grid 
    else fullcolor=0xFF0000;    // unknown area
    XSetForeground(myDisplay, myGc, fullcolor);

    //XDrawRectangle(myDisplay, myWindow, myGc, x1-lenx,y1-leny,lenx*2,leny*2);
    XDrawLine(myDisplay, myWindow, myGc, x1, y1, x2, y2);
    XDrawLine(myDisplay, myWindow, myGc, x1, y1, x4, y4);
    XDrawLine(myDisplay, myWindow, myGc, x3, y3, x2, y2);
    XDrawLine(myDisplay, myWindow, myGc, x3, y3, x4, y4);

}

void T1Draw::DrawGridMap(T1Map2 *map2) {
    //printf("T1Draw-DrawGridMap begin......\n");
    int maxGridX,maxGridY,sizeGrid,minX,minY;
    maxGridX = map2->GetLength('x');
    maxGridY = map2->GetLength('y');
    sizeGrid = map2->GetLength('g');
    minX = map2->GetPoint('i',0).lon;
    minY = map2->GetPoint('i',0).lat;
    for ( int i=0; i<maxGridX; i++ ) {
        for ( int j=0; j<maxGridY; j++ ) {
            DrawGrid( i, j, sizeGrid, map2->myGrid[i][j], minX, minY );
        }
    }
    //printf("T1Draw-DrawGridMap end. %d,%d\n", maxGridX, maxGridY);

    // draw Tag's position
    XSetForeground(myDisplay, myGc, 0x7F7FCF);
    int length = map2->myMapAprilTag.size();
    //printf("T1Draw-DrawTag begin, length=%d\n", length);
    for ( int i=0; i<length; i++ ) {
        MapPoint tmppoint = map2->GetPoint2('A',i);
        DrawTag(tmppoint);
    }

}

void T1Draw::DrawTag(MapPoint point) {
    int tmpx,tmpy,x1,y1,x2,y2,x3,y3,x4,y4,lenx,leny;
    int lenmin=15;
    //printf("T1Draw-DrawTag begin, point:(%lld,%lld)\n", point.lon, point.lat);
    tmpx = point.lon;
    tmpy = point.lat;
    x1=DrawGetRotaX(tmpx, tmpy, 0);
    y1=DrawGetRotaY(tmpy, 0, 0);
    lenx = lenmin * rotation.scale_x;
    leny = lenmin * rotation.scale_y;
    XDrawRectangle(myDisplay, myWindow, myGc, x1-lenx/2,y1-leny/2,lenx,leny);
}

void T1Draw::DrawRoutePlan2(T1Map2 *map) {
    //printf("T1Draw-DrawRoutePlan2 begin.\n");
    int count,tmpx,tmpy,x1,y1,x0,y0;

    XSetForeground(myDisplay, myGc, 0x3030F0);
    count = map->GetLength('2');
    //printf("T1Draw-DrawRoutePlan2 loop[%d].\n", count);
    for ( int i=0; i<count; i++ ) {
        //DrawMapBorder(painter,map->GetPoint('P',i),i);
        tmpx = map->GetPoint2('2',i).lon;// + map->GetLength('g')/2;
        tmpy = map->GetPoint2('2',i).lat;// + map->GetLength('g')/2;
        x1=DrawGetRotaX(tmpx, tmpy, 0);
        y1=DrawGetRotaY(tmpy, 0, 0);
        //printf("T1Draw-DrawRoutePlan2 loop(%d), point(%d,%d)(%d,%d).\n", i, tmpx, tmpy, x1, y1);
        //painter->setPen(QPen(Qt::green,1));
        XDrawRectangle(myDisplay, myWindow, myGc, x1-2, y1-2, 5, 5);
        /*if ( i%2 == 1 ) {
            //painter->setPen(QPen(Qt::darkGreen,1));
            //XDrawLine(myDisplay, myWindow, myGc, x0, y0, x1, y1);
            XSetForeground(myDisplay, myGc, 0x3030F0);
        } else {
            XSetForeground(myDisplay, myGc, 0x30F030);
        }*/
        if ( i > 0 ) XDrawLine(myDisplay, myWindow, myGc, x0, y0, x1, y1);
        x0 = x1; y0 = y1;
    }
}

void T1Draw::DrawRoutePlanLine(T1Map2 *map) {
    //printf("T1Draw-DrawRoutePlan2 begin.\n");
    int count,tmpx,tmpy,x1,y1,x0,y0;

    count = map->GetLength('L');
    //printf("T1Draw-DrawRoutePlanLine loop[%d].\n", count);
    for ( int i=0; i<count; i++ ) {
        //DrawMapBorder(painter,map->GetPoint('P',i),i);
        tmpx = map->GetPoint2('L',i).lon;// + DIFFX;// - map->GetPoint('b',0).lon;
        tmpy = map->GetPoint2('L',i).lat;// + DIFFY;// - map->GetPoint('b',0).lat + DIFFY;
        x0=DrawGetRotaX(tmpx, tmpy, 0);
        y0=DrawGetRotaY(tmpy, 0, 0);
        XSetForeground(myDisplay, myGc, 0x30C0C0);
        XDrawRectangle(myDisplay, myWindow, myGc, x0-2, y0-2, 5, 5);
        tmpx = map->GetPoint2('l',i).lon;// + DIFFX;// - map->GetPoint('b',0).lon;
        tmpy = map->GetPoint2('l',i).lat;// + DIFFY;// - map->GetPoint('b',0).lat + DIFFY;
        x1=DrawGetRotaX(tmpx, tmpy, 0);
        y1=DrawGetRotaY(tmpy, 0, 0);
        //printf("T1Draw-DrawRoutePlanLine loop(%d), point(%d,%d)(%d,%d).\n", i, x0, y0, x1, y1);
        XSetForeground(myDisplay, myGc, 0xC0C030);
        XDrawRectangle(myDisplay, myWindow, myGc, x1-2, y1-2, 5, 5);
        XDrawLine(myDisplay, myWindow, myGc, x0, y0, x1, y1);
    }
}

void T1Draw::DrawRoutePlan(T1Map *map) {
    int count,tmpx,tmpy,x1,y1,x0,y0;

    XSetForeground(myDisplay, myGc, 0xC0E000);
    count = map->GetLength('P');
    for ( int i=0; i<count; i++ ) {
        //DrawMapBorder(painter,map->GetPoint('P',i),i);
        tmpx = map->GetPoint('P',i).lon;// + DIFFX;// - map->GetPoint('b',0).lon;
        tmpy = map->GetPoint('P',i).lat;// + DIFFY;// - map->GetPoint('b',0).lat + DIFFY;
        x1=DrawGetRotaX(tmpx, tmpy, 0);
        y1=DrawGetRotaY(tmpy, 0, 0);
        //painter->setPen(QPen(Qt::green,1));
        XDrawRectangle(myDisplay, myWindow, myGc, x1-2, y1-2, 5, 5);
        if ( i > 0 ) {
            //painter->setPen(QPen(Qt::darkGreen,1));
            XDrawLine(myDisplay, myWindow, myGc, x0, y0, x1, y1);
        }
        x0 = x1; y0 = y1;
    }
}

void T1Draw::DrawMap(T1Map *map) {
    int count,tmpx,tmpy,x1,y1,x0,y0;

    // draw routePlan
    //XSetForeground(myDisplay, myGc, 0x0000FF);

    // draw base
    XSetForeground(myDisplay, myGc, 0x00AF00);
    DrawMapBase(map);//map->GetPoint('b',0), map->GetPoint('r',0));
    // draw Min & Max
    XSetForeground(myDisplay, myGc, 0xFFAFFF);
    DrawMaxMin(map->GetPoint('a',0),map->GetPoint('i',0));
    // draw remote point
    XSetForeground(myDisplay, myGc, 0x00AFAF);
    count = map->GetLength('R');
    for ( int i=1; i<count; i++ ) {
        DrawMapTrail(map->GetPoint('R',i));
    }
    // draw border point
    XSetForeground(myDisplay, myGc, 0x00CF00);
    count = map->GetLength('B');
    for ( int i=0; i<count; i++ ) {
        DrawMapBorder(map->GetPoint('B',i));
    }
    
    // draw RoutePlan point
    if ( map->GetLength('L') != -1 ) {
        DrawRoutePlan2((T1Map2 *)map);
        DrawRoutePlanLine((T1Map2 *)map);
    } else {
        DrawRoutePlan(map);
    }


    //myFlush();

}

void T1Draw::DrawMaxMin(MapPoint max, MapPoint min) {
    int tmpx,tmpy,x1,y1,x2,y2,x3,y3,x4,y4,lenx,leny;
    int lenmin=60, lenmax=80;
//printf("T1Draw-DrawMapMaxMin begin, Max:(%lld,%lld), Min:(%lld,%lld)\n", max.lon, max.lat, min.lon, min.lat);
    // draw min
    tmpx = min.lon;// - base.lon;// + DIFFX;// - map->GetPoint('b',0).lon;
    tmpy = min.lat;// - base.lat;// + DIFFY;// - map->GetPoint('b',0).lat;
    x1=DrawGetRotaX(tmpx, tmpy, 0);
    y1=DrawGetRotaY(tmpy, 0, 0);
    lenx = lenmin * rotation.scale_x;
    leny = lenmin * rotation.scale_y;
    XDrawRectangle(myDisplay, myWindow, myGc, x1-lenx/2,y1-leny/2,lenx,leny);
    XDrawLine(myDisplay, myWindow, myGc, x1-lenx/2, y1, x1+lenx/2, y1);
    XDrawLine(myDisplay, myWindow, myGc, x1, y1-leny/2, x1, y1+leny/2);
    // draw max
    tmpx = max.lon;// - base.lon;// + DIFFX;// - map->GetPoint('b',0).lon;
    tmpy = max.lat;// - base.lat;// + DIFFY;// - map->GetPoint('b',0).lat;
    x2=DrawGetRotaX(tmpx, tmpy, 0);
    y2=DrawGetRotaY(tmpy, 0, 0);
    lenx = lenmax * rotation.scale_x;
    leny = lenmax * rotation.scale_y;
    XDrawRectangle(myDisplay, myWindow, myGc, x2-lenx/2,y2-leny/2,lenx,leny);
    XDrawLine(myDisplay, myWindow, myGc, x2-lenx/2, y2, x2+lenx/2, y2);
    XDrawLine(myDisplay, myWindow, myGc, x2, y2-leny/2, x2, y2+leny/2);
    // draw outer box
    tmpx = max.lon; tmpy = min.lat;
    x3=DrawGetRotaX(tmpx, tmpy, 0);    y3=DrawGetRotaY(tmpy, 0, 0);
    tmpx = min.lon; tmpy = max.lat;
    x4=DrawGetRotaX(tmpx, tmpy, 0);    y4=DrawGetRotaY(tmpy, 0, 0);
    XDrawLine(myDisplay, myWindow, myGc, x1, y1, x3, y3);
    XDrawLine(myDisplay, myWindow, myGc, x1, y1, x4, y4);
    XDrawLine(myDisplay, myWindow, myGc, x2, y2, x3, y3);
    XDrawLine(myDisplay, myWindow, myGc, x2, y2, x4, y4);

}

void T1Draw::DrawMapBase(T1Map *map) {  //MapPoint base, MapPoint baseRobo)
    int tmpx,tmpy,x1,y1,lenx,leny;
//printf("T1Draw-DrawMapBase begin: (%lld,%lld)\n",base.lon,base.lat);
    //painter->drawRect(base.lat-5,base.lon-5,10,10);
    MapPoint base, tmpPoint;
    base = map->GetPoint('b',0);
    tmpx = base.lon - base.lon;// + DIFFX;
    tmpy = base.lat - base.lat;// + DIFFY;
    x1=DrawGetRotaX(tmpx, tmpy, 0);
    y1=DrawGetRotaY(tmpy, 0, 0);
    lenx = 100 * rotation.scale_x;
    leny = 100 * rotation.scale_y;
    XDrawRectangle(myDisplay, myWindow, myGc, x1-lenx/2,y1-leny/2,lenx,leny);
    char tmpstr[220];
    sprintf(tmpstr, "mapBase :(%lld,%lld)", base.lon, base.lat);
    XDrawString(myDisplay, myWindow, myGc, 15, 70, tmpstr, strlen(tmpstr));
    sprintf(tmpstr, "map:[%s]", MapName);
    XDrawString(myDisplay, myWindow, myGc, 15, 55, tmpstr, strlen(tmpstr));

}

void T1Draw::DrawMapBorder(MapPoint point) {
    int tmpx,tmpy,x1,y1;
    
    tmpx = point.lon;// + DIFFX;// - map->GetPoint('b',0).lon;
    tmpy = point.lat;// + DIFFY;// - map->GetPoint('b',0).lat;
    x1=DrawGetRotaX(tmpx, tmpy, 0);
    y1=DrawGetRotaY(tmpy, 0, 0);
    XDrawPoint(myDisplay, myWindow, myGc, x1, y1);
    
}

void T1Draw::DrawMapTrail(MapPoint point) {  // the RTK's trail when user remoto control, used to log map
    int tmpx,tmpy,x1,y1;
    //painter->drawRect(base.lat-5,base.lon-5,10,10);
    tmpx = point.lon;// + DIFFX;// - base.lon；
    tmpy = point.lat;// + DIFFY;// - base.lat;
    //printf("T1Draw-DrawMapTrail_ begin, (%d,%d)\n", tmpx, tmpy);
    x1=DrawGetRotaX(tmpx, tmpy, 0);
    y1=DrawGetRotaY(tmpy, 0, 0);
    if ( point.fixed == 4 ) {
        XDrawRectangle(myDisplay, myWindow, myGc, x1-1, y1-1, 3, 3);
    } else if ( point.fixed == 5 ) { //should be 5
        XSetForeground(myDisplay, myGc, 0x909050);
        //painter->setPen(QPen(Qt::darkYellow,1,Qt::DotLine));
        //painter->drawRect(x1-3,y1-3,7,7);
        XDrawArc(myDisplay, myWindow, myGc, x1-8, y1-8, 16, 16, 0, 360*64);
        //XDrawRectangle(myDisplay, myWindow, myGc, x1-5, y1-5, 11, 11);
    } else {
        XSetForeground(myDisplay, myGc, 0x90FF90);
        //painter->setPen(QPen(Qt::darkGreen,1,Qt::DotLine));
        //painter->drawRect(x1-5,y1-5,11,11);
        //QRectF rectangle(x1-10, y1-10, 20, 20 ); painter->drawArc(rectangle, 10 * 16, 360 * 16);
        XDrawArc(myDisplay, myWindow, myGc, x1-15, y1-15, 30, 30, 0, 360*64);
    }
    //painter->setPen(QPen(Qt::magenta,1));
    //painter->drawRect(x1,y1,2,2);
}

int T1Draw::DrawGetRotaX(int x,int y,int noShift) {
    int rotaX,dx;
    dx = rotation.dxsin * rotation.scale_y * y;
    rotaX = rotation.scale_x * ( dx + x );
    if ( noShift != 1 )
        rotaX += rotation.shift_x + winwidth/2;
    else
        rotaX += winwidth/2;//60;
//printf("T1Draw-DrawGetRotaX, in:%d,%d,z,%d, out:%d\n",x,y,noShift,rotaX);
    return rotaX;
}

int T1Draw::DrawGetRotaY(int y,int z, int noShift) {
    int rotaY,dy;
    dy = rotation.dycos * rotation.scale_y * y;
    rotaY = rotation.scale_z * ( dy + z ) + winheight;
    if ( noShift != 1 )
        rotaY -= rotation.shift_y + winheight/2;
    else
        rotaY -= winheight/2;
//printf("T1Draw-DrawGetRotaY, in:x,%d,%d,%d, out:%d\n",y,z,noShift,rotaY);
    return rotaY;
}

void T1Draw::SetMapName(const char * name) {
    strcpy(MapName, name);
}

void T1Draw::SetWinSize(int win_w, int win_h, int l_w, int l_h) {
    winwidth = win_w;
    winheight = win_h;
    lawnwidth = l_w;
    lawnheight = l_h;
}

void T1Draw::SetShift(int shx, int shy) {
    rotation.shift_x = shx;
    rotation.shift_y = shy;
}

void T1Draw::SetScale(float sx, float sy, float sz) {
    rotation.scale_x = sx;
    rotation.scale_y = sy;
    rotation.scale_z = sz;
}

void T1Draw::SetRotate(float ax, float ay, float az) {
    rotation.angle_x = ax;
    rotation.angle_y = ay;
    rotation.angle_z = az;
    rotation.dxsin = sin((rotation.angle_y*3.1415926)/180);
    rotation.dycos = cos((rotation.angle_y*3.1415926)/180);
//    printf("T1Draw-SetRotate: %.3f, %.3f, %.3f, %.3f, %.3f\n", ax, ay, az, rotation.dxsin, rotation.dycos);
}
