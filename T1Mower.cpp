#include "T1Mower.hpp"
#include "T1Utils.hpp"

//#include <QFile>
//#include <QTextStream>
//#include <QDebug>
#include <math.h>
#include <fstream>
#include <iostream>

T1Mower::T1Mower()
{
    MoveRotate(0.0,0.0);
}

T1Mower::~T1Mower()
{
}

void T1Mower::SetPBangs(void)
{/*
    pBangs.clear();

    MyPointBang tmp_p;

    for ( int i=0; i<body.length(); i++ ) {
        if ( body[i].radius == 0 )
            continue;
        for ( int j=0; j<=body[i].steps; j++ ) {
            MyPoint p1,p2;
            p1.x =  body[i].begin.x + j*body[i].stepx;
            p1.y =  body[i].begin.y + j*body[i].stepy;
            p1.z =  body[i].begin.z + j*body[i].stepz;
            //p2 = p1;
            p2 = RotateAxis(body[i].type,p1);
            //tmp_p.type = body[i].type;
            tmp_p.r = body[i].radius;
            tmp_p.x = p2.x + move.x;
            tmp_p.y = p2.y + move.y;
            tmp_p.z = p2.z + move.z;
            pBangs.append(tmp_p);
            //qDebug() << filename << i << j << p1.x << p1.y <<p1.z << body[i].stepx;
        }
    }
    */
    //qDebug() <<
}

void T1Mower::init(const char * name)
{
    GetDefault();

    sprintf(filename,"%s", name);
    std::ifstream filein(filename);
    if (!filein.is_open() )
        return;
    std::cout << "T1Mower-init, read: " << filename << std::endl;

    std::string line;
    std::vector<std::string> list1;
    MyLine tmpmowerline;
    int tmptype = 0;

    //body.clear();
    int count = 0;
    while ( std::getline(filein,line) ) {
        list1 = T1Utils::split(line,' ');
        //std::cout << "reading: " << list1.size() << "[" << line << "]" << std::endl;
        if ( line.substr(0,2) == "//" ) continue;
        if ( line.substr(0,5) == "#name" ) {
             sprintf(filename,"%s", list1[1].c_str() );//line.remove(0,6);
             continue;
        }
        if ( line.substr(0,5) == "#type" ) {
             tmptype = std::stoi(list1[1]);//line.remove(0,6)).toInt();
             printf("T1Mower-init, read type: %d\n",tmptype);
             continue;
        }
        if ( line.substr(0,5) == "#axis" ) {
            std::vector<std::string> point_axis = T1Utils::split(list1[2],',');//, QString::SkipEmptyParts);
            if ( list1[1] == "horizontal" ) {
                axisH.x = std::stoi(point_axis[0]);
                axisH.y = std::stoi(point_axis[1]);
                axisH.z = std::stoi(point_axis[2]);
                //std::cout << "axisH:" << axisH.x << axisH.y << axisH.z;
            } else if ( list1[1] == "vertical"  ) {
                axisV.x = std::stoi(point_axis[0]);
                axisV.y = std::stoi(point_axis[1]);
                axisV.z = std::stoi(point_axis[2]);
                //std::cout << "axisV:" << axisV.x << axisV.y << axisV.z;
            }
            continue;
        }

        //std::cout << list1.size() << "[" << line << "]" << std::endl;
        if ( list1.size() != 3 ) continue;
        if ( count == 0 ) body.clear();  // clear default if the file has 1 valid line;
        count ++;

        std::vector<std::string> point_begin = T1Utils::split(list1[1],',');//, QString::SkipEmptyParts);
        std::vector<std::string> point_end = T1Utils::split(list1[2],',');//, QString::SkipEmptyParts);

        tmpmowerline.type = tmptype;
        tmpmowerline.radius = std::stoi(list1[0]);
        tmpmowerline.begin.x = std::stoi(point_begin[0]);
        tmpmowerline.begin.y = std::stoi(point_begin[1]);
        tmpmowerline.begin.z = std::stoi(point_begin[2]);
        tmpmowerline.end.x = std::stoi(point_end[0]);
        tmpmowerline.end.y = std::stoi(point_end[1]);
        tmpmowerline.end.z = std::stoi(point_end[2]);
        if ( 0 != tmpmowerline.radius ) {
            float dx,dy,dz,d;
            //dx=fabs(tmpmowerline.begin.x-tmpmowerline.end.x);
            //dy=fabs(tmpmowerline.begin.y-tmpmowerline.end.y);
            //dz=fabs(tmpmowerline.begin.z-tmpmowerline.end.z);
            dx = tmpmowerline.begin.x - tmpmowerline.end.x;
            dy = tmpmowerline.begin.y - tmpmowerline.end.y;
            dz = tmpmowerline.begin.z - tmpmowerline.end.z;
            d = sqrt(dx*dx+dy*dy+dz*dz);
            tmpmowerline.steps = d/tmpmowerline.radius + 1;
            tmpmowerline.stepx = float(tmpmowerline.end.x-tmpmowerline.begin.x)/tmpmowerline.steps;
            tmpmowerline.stepy = float(tmpmowerline.end.y-tmpmowerline.begin.y)/tmpmowerline.steps;
            tmpmowerline.stepz = float(tmpmowerline.end.z-tmpmowerline.begin.z)/tmpmowerline.steps;
            //qDebug() << "devide: " << dx << dy << dz << d << tmpmowerline.radius <<tmpmowerline.steps;
        }
        body.push_back(tmpmowerline);
        //qDebug() << tmptype;
    }
    
    //std::cout << "T1Mower-init: reading upt end." << count << std::endl;//\nbody:" << body.length();
    filein.close();
    //SetPBangs();
}

void T1Mower::GetDefault() // default mower is just a triangle
{
    std::string lines[] = { // default mower is just a triangle, only 3 lines
        "0 -20,20,0 -20,-20,0",
        "0 -20,20,0 39,0,0",
        "0 -20,-20,0 39,0,0"
    };
    std::vector<std::string> list1;
    MyLine tmpmowerline;

    body.clear();
    for ( int i=0; i<3; i++ ) {
        list1 = T1Utils::split(lines[i],' ');
        std::vector<std::string> point_begin = T1Utils::split(list1[1],',');//, QString::SkipEmptyParts);
        std::vector<std::string> point_end = T1Utils::split(list1[2],',');//, QString::SkipEmptyParts);

        tmpmowerline.type = 1; // horizontal rotate
        tmpmowerline.radius = std::stoi(list1[0]);
        tmpmowerline.begin.x = std::stoi(point_begin[0]);
        tmpmowerline.begin.y = std::stoi(point_begin[1]);
        tmpmowerline.begin.z = std::stoi(point_begin[2]);
        tmpmowerline.end.x = std::stoi(point_end[0]);
        tmpmowerline.end.y = std::stoi(point_end[1]);
        tmpmowerline.end.z = std::stoi(point_end[2]);

        body.push_back(tmpmowerline);
    }
}

void T1Mower::GetSet(MyPoint *move, rtkPoint rtk1, int Diff)
{
    if ( Diff==0 || meDiff==-99999 ) {
        ;
    } else if ( Diff==1 ) { // mower must on trail and on earth
        rtk1.x = fixRTK1.x/10.0 + meDiff/10.0;
        rtk1.z = fixRTK1.z/10.0;
    } else if ( Diff==2 ) {  // not used now
        rtk1.x = 0;//T1Mowers[i].fixRTK1.x;
        rtk1.z = 0;//T1Mowers[i].fixRTK1.z;
    }

    move->x = (int)(rtk1.x*10) - fixRTK1.x;
    move->y = (int)(rtk1.y*10) - fixRTK1.y;
    move->z = (int)(rtk1.z*10) - fixRTK1.z;
    //qDebug() << "T1Mower-GetSet step1: " << rtk1.x << rtk1.y << rtk1.z << fixRTK1.x << fixRTK1.y << fixRTK1.z;
    //qDebug() << "T1Mower-GetSet step2: " << IDdevice << move->x/10.0 << move->y/10.0 << move->z/10.0;
}

void T1Mower::GetRotate(MyPoint *move, rtkPoint *Rotate, rtkPoint rtk1, rtkPoint rtk2, int Diff)
{
    // 2022.5.11 new:
    //float r1 = sqrt( fixRTK1.x*fixRTK1.x + fixRTK1.y*fixRTK1.y);
    //float r2 = sqrt( fixRTK2.x*fixRTK2.x + fixRTK2.y*fixRTK2.y);
    float r1 = fixRTK1.x/10.0;
    //float r2 = fixRTK2.x;
    float r = fixRTK2.x/10.0 - r1; // this will not be zero
    //float r  = r2 - r1;
    float dx = rtk2.x - rtk1.x;
    float dy = rtk2.y - rtk1.y;
    float dr = sqrt ( dx*dx + dy*dy );
    //float dz = rtk2.z - rtk1.z;
    //float rV = asin(dz/r);
    if ( dr > r ) dr=r;
    float rV = acos(dr/r);
    Rotate->y = rV*180/3.14159265;
    //qDebug() << "T1Mower-GetRotate step2: " << dx << dy << dr << r << rV << Rotate->y;
    float rH = atan2( dy,dx ); // dx may be zero
    float x0 = rtk1.x - r1*sin(rH);
    float y0 = rtk1.y - r1*cos(rH);
    // diff adjust
    if ( Diff==0 || meDiff==-99999 ) {
        ;
    } else if ( Diff==1 ) {
        x0 = meDiff/10.0;
        rtk1.z = fixRTK1.z/10.0;
    }
    // end of 2022.5.11 new


    move->x = (int)(x0*10);
    move->y = (int)(y0*10);
    move->z = (int)(rtk1.z*10) - fixRTK1.z;
    //float rH = atan( (rtk2.x-x0) / (rtk2.y-y0) );
    Rotate->x = rH*180/3.14159265;
    //QString tmpstr1,tmpstr2;
    //tmpstr1.sprintf("T1Mower-GetRotate step3: xy1: %4.0f,%4.0f,%4.0f xy2: %4.0f,%4.0f,%4.0f xy0:%4.0f,%4.0f"
    //           ,rtk1.x,rtk1.y,rtk1.z,rtk2.x,rtk2.y,rtk2.z,x0,y0);
    //tmpstr2.sprintf("fixRTK: %d,%d          diff:  %4.0f,%4.0f,%4.0f,%4.0f         result: %4.1f,%4.1f,%4.1f,%4.2f,%4.2f"
    //                ,fixRTK1.x,fixRTK2.x,dx,dy,dr,r,move->x,move->y,move->z,Rotate->x,Rotate->y);
    //qDebug() << tmpstr1;
    //qDebug() << tmpstr2;
    //qDebug() << "T1Mower-GetRotate step3: xy1:" << rtk1.x << rtk1.y << rtk1.z << " xy2:" << rtk2.x << rtk2.y <<rtk2.z
             //<< " xy0: " << x0 << y0
             //<< " result:" << move->x << move->y << move->z << Rotate->x << Rotate->y; // rH*180/3.14159265;

}

void T1Mower::MoveSet(int mx,int my,int mz)
{
    //qDebug() << "MoveSet: " << mx << my << mz;
    move.x = mx;
    move.y = my;
    move.z = mz;
    //SetPBangs();
}

void T1Mower::MoveRotate(float rh,float rv)
{
    rotationH = rh;
    rHcos = sin( 3.14159265*rotationH/180 );
    rHsin = cos( 3.14159265*rotationH/180 );
    rotationV = rv;
    rVsin = sin( 3.14159265*rotationV/180 );
    rVcos = cos( 3.14159265*rotationV/180 );
    //printf("T1Mower-MoveRotate: %d, %.3f, %.3f\n ",IDdevice, rh, rv);
    //SetPBangs();
}

MyPoint T1Mower::RotateAxis(int type, MyPoint point)
{
    MyPoint p1;
    float dx1,dy1,dx2,dy2,dz1,dz2;
    p1.x = point.x;
    p1.y = point.y;
    p1.z = point.z;

    if ( type == 1 ) {
        dx1 = point.x - axisH.x;
        dy1 = point.y - axisH.y;
        dx2 = dx1*rHcos - dy1*rHsin;
        dy2 = dx1*rHsin + dy1*rHcos;
        p1.x = axisH.x + dx2;
        p1.y = axisH.y + dy2;
        //qDebug() << "T1Mower-RptataAxis step1: " << IDdevice << rotationH << type << p1.x << p1.y << p1.z << rHcos << rHsin << axisH.x << axisH.y;
    } else if ( type==2 ) {
        dx1 = point.x - axisV.x;
        dz1 = point.z - axisV.z;
        dx2 = dx1*rVcos - dz1*rVsin;
        dz2 = dx1*rVsin + dz1*rVcos;
        p1.x = axisV.x + dx2;
        p1.z = axisV.z + dz2;
        dx1 = p1.x - axisH.x;
        dy1 = point.y - axisH.y;
        dx2 = dx1*rHcos - dy1*rHsin;
        dy2 = dx1*rHsin + dy1*rHcos;
        p1.x = axisH.x + dx2;
        p1.y = axisH.y + dy2;
        //qDebug() << "T1Mower-RptataAxis step2: " << IDdevice << rotationH << type << p1.x << p1.y << p1.z << rHcos << rHsin << axisH.x << axisH.y;
    }
//printf("T1Mower-RotateAxis: %d,in:%d,%d,%d,out:%d,%d,%d\n",type,point.x,point.y,point.z,p1.x,p1.y,p1.z);
    return p1;
}

/*
void T1Mower::BangDetect(int count, T1Mower *neighbour, QVector<MyAlarm> *alarm)
{
    //int i,j,x1,y1,z1,mmx,mmy,mmz;
    int alarm_blue = 399;
    MyPoint p1,p2;
    for ( int i=0; i<body.length(); i++ ) {
        //qDebug() << "BangDetect: " << i << mmx << mmy << mmz << body[i].radius;
        if ( 0 == body[i].radius ) continue;
        for ( int j=0; j<=body[i].steps; j++ ) {
            int x1,y1,z1;
            p1.x =  body[i].begin.x + j*body[i].stepx;
            p1.y =  body[i].begin.y + j*body[i].stepy;
            p1.z =  body[i].begin.z + j*body[i].stepz;
            p2 = RotateAxis(body[i].type,p1);
            x1=p2.x; y1=p2.y; z1=p2.z;
            //neighbour->BangDetectTwo(count,x1,y1,z1,mmx,mmy,mmz);
            //qDebug() << "BangDetect: " << count << i << j << x1 << y1 << z1 << mmx << mmy << mmz << body[i].radius;
            MyAlarm tmpalarm;
            int dmin = alarm_blue;
            for ( int i2=0; i2<body.length(); i2++ ) {
                //qDebug() << "BangDetect: " << count << i << mmx << mmy << mmz << body[i].radius;
                if ( 0 == neighbour->body[i2].radius ) continue;
                for ( int j2=0; j2<=neighbour->body[i2].steps; j2++ ) {
                    int x2,y2,z2;
                    p1.x =  neighbour->body[i2].begin.x + j2*neighbour->body[i2].stepx;
                    p1.y =  neighbour->body[i2].begin.y + j2*neighbour->body[i2].stepy;
                    p1.z =  neighbour->body[i2].begin.z + j2*neighbour->body[i2].stepz;
                    p2 = RotateAxis(neighbour->body[i2].type,p1);
                    x2=p2.x; y2=p2.y; z2=p2.z;
                    int dx = x1 - x2 + move.x - neighbour->move.x;
                    int dy = y1 - y2 + move.y - neighbour->move.y;
                    int dz = z1 - z2 + move.z - neighbour->move.z;
                    if ( dx>alarm_blue || dx<(-1*alarm_blue) ) continue;
                    if ( dy>alarm_blue || dy<(-1*alarm_blue) ) continue;
                    if ( dz>alarm_blue || dz<(-1*alarm_blue) ) continue;
                    float d = sqrt(dx*dx+dy*dy+dz*dz);
                    if ( ( d - body[i].radius - neighbour->body[i2].radius ) >= dmin ) continue;
                    dmin = ( d - body[i].radius - neighbour->body[i2].radius );
                    //if ( d >= dmin ) continue;
                    //dmin = ( d );
                    tmpalarm.x1 = x1 + move.x;
                    tmpalarm.y1 = y1 + move.y;
                    tmpalarm.z1 = z1 + move.z;
                    tmpalarm.r1 = body[i].radius;
                    tmpalarm.x2 = x2 + neighbour->move.x;
                    tmpalarm.y2 = y2 + neighbour->move.y;
                    tmpalarm.z2 = z2 + neighbour->move.z;
                    tmpalarm.r2 = neighbour->body[i2].radius;
                    tmpalarm.distance = d;
                    //qDebug() << "BangDetectTwo: " << count << i2 << j2 << d;
                }
                //qDebug() << "circle: " << count << i << mmx << mmy << mmz << body[i].radius;
            }
            if ( dmin < alarm_blue )
                alarm->append(tmpalarm);
        }
        //qDebug() << "circle: " << count << i << body[i].radius;
    }
    if ( alarm->length() > 0 )
        qDebug() << "DetectBang: " << count << alarm->length() << alarm->data();
}
*/
/*
std::vector<std::string> T1Mower::split(const std::string& str, char ch) {
    std::vector<std::string> tokens;
    std::string token;
    bool inToken = false;
    
    for (char c : str) {
        if ( c == ch ) {
            if (inToken) {
                tokens.push_back(token);
                token.clear();
                inToken = false;
            }
        } else {
            token += c;
            inToken = true;
        }
    }
    
    // add the last token (if exist)
    if (inToken) {
        tokens.push_back(token);
    }
    
    return tokens;
}
*/