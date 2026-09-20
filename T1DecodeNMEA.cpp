#include "T1DecodeNMEA.hpp"
#include <math.h>
#include <string>

//#include <QDebug>

#define MAXFIELD    32                  /* max number of fields in a record */
#define PI          3.1415926535897932	/* pi */
#define D2R         (PI/180.0)          /* deg to rad */
#define RE_WGS84    6378137.0           /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563) /* earth flattening (WGS84) */
/*
T1DecodeNMEA::T1DecodeNMEA()
{

}

T1DecodeNMEA::~T1DecodeNMEA()
{

}
*/

/* multiply matrix -----------------------------------------------------------*/
extern void matmul(const char *ttr, int n, int k, int m, double alpha,
                   const double *A, const double *B, double beta, double *C)
{
    double d;
    int i,j,x,f=ttr[0]=='N'?(ttr[1]=='N'?1:2):(ttr[1]=='N'?3:4);

    for (i=0;i<n;i++) for (j=0;j<k;j++) {
        d=0.0;
        switch (f) {
            case 1: for (x=0;x<m;x++) d+=A[i+x*n]*B[x+j*m]; break;
            case 2: for (x=0;x<m;x++) d+=A[i+x*n]*B[j+x*k]; break;
            case 3: for (x=0;x<m;x++) d+=A[x+i*m]*B[x+j*m]; break;
            case 4: for (x=0;x<m;x++) d+=A[x+i*m]*B[j+x*k]; break;
        }
        if (beta==0.0) C[i+j*n]=alpha*d; else C[i+j*n]=alpha*d+beta*C[i+j*n];
    }
}

/* ecef to local coordinate transfromation matrix ------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *E        O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
extern void xyz2enu(const double *pos, double *E)
{
    double sinp=sin(pos[0]),cosp=cos(pos[0]),sinl=sin(pos[1]),cosl=cos(pos[1]);

    E[0]=-sinl;      E[3]=cosl;       E[6]=0.0;
    E[1]=-sinp*cosl; E[4]=-sinp*sinl; E[7]=cosp;
    E[2]=cosp*cosl;  E[5]=cosp*sinl;  E[8]=sinp;
}
/* transform ecef vector to local tangental coordinate -------------------------
* transform ecef vector to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *r        I   vector in ecef coordinate {x,y,z}
*          double *e        O   vector in local tangental coordinate {e,n,u}
* return : none
*-----------------------------------------------------------------------------*/
extern void ecef2enu(const double *pos, const double *r, double *e)
{
    double E[9];

    xyz2enu(pos,E);
    matmul("NN",3,1,3,1.0,E,r,0.0,e);
}

/* transform geodetic to ecef position -----------------------------------------
* transform geodetic position to ecef position
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *r        O   ecef position {x,y,z} (m)
* return : none
* notes  : WGS84, ellipsoidal height
* #define RE_WGS84    6378137.0           earth semimajor axis (WGS84) (m)
* #define FE_WGS84    (1.0/298.257223563)     earth flattening (WGS84)
*-----------------------------------------------------------------------------*/
//void T1DecodeNMEA::
extern void pos2ecef(const double *pos, double *r)
{
    //std::cout << "T1DecodeNMEA-pos2ecef step1: pos: " << pos[0] << pos[1] << pos[2];
    double sinp=sin(pos[0]),cosp=cos(pos[0]),sinl=sin(pos[1]),cosl=cos(pos[1]);
    double e2=FE_WGS84*(2.0-FE_WGS84),v=RE_WGS84/sqrt(1.0-e2*sinp*sinp);
    //std::cout << "T1DecodeNMEA-pos2ecef step2: e2=" << e2;

    r[0]=(v+pos[2])*cosp*cosl;
    r[1]=(v+pos[2])*cosp*sinl;
    r[2]=(v*(1.0-e2)+pos[2])*sinp;
    //std::cout << "T1DecodeNMEA-pos2ecef step2: r: " << r[0] << r[1] << r[2];
}

void T1DecodeNMEA::pos2enu(rtkPoint *rtk_e, rtkPoint rtk_r, std::vector<std::string> list)//QStringList list)
{
    double pos[3],r[3],e[3];
    nmea_raw_t2 raw;

    r[0] = rtk_r.x;
    r[1] = rtk_r.y;
    r[2] = rtk_r.z;
    //std::cout << "T1DecodeNMEA-pos2enu step0:" <<list[0];// <<*/ raw.lat << raw.lon << raw.alt;

    decode_nmeaGGA(&raw,list);
    //std::cout << "T1DecodeNMEA-pos2enu step1:" <</* list <<*/ raw.lat << raw.lon << raw.alt;
    pos[0] = raw.lat;
    pos[1] = raw.lon;
    pos[2] = raw.alt;

    ecef2enu(pos,r,e);
    rtk_e->x = e[0];
    rtk_e->y = e[1];
    rtk_e->z = e[2];
    //std::cout << "T1DecodeNMEA-pos2enu step3:" <</* list <<*/ e[0] << e[1] <<e[2];// << raw.lon << raw.alt;
}

void T1DecodeNMEA::lla2enu(rtkPoint *rtk_e, rtkPoint rtk_r, rtkPoint raw)
{
    double pos[3],r[3],e[3];
    //nmea_raw_t2 raw;

    r[0] = rtk_r.x;
    r[1] = rtk_r.y;
    r[2] = rtk_r.z;
    //std::cout << "T1DecodeNMEA-pos2enu step0:" <<list[0];// <<*/ raw.lat << raw.lon << raw.alt;

    //decode_nmeaGGA(&raw,list);
    //std::cout << "T1DecodeNMEA-pos2enu step1:" <</* list <<*/ raw.lat << raw.lon << raw.alt;
    pos[0] = raw.x * D2R;
    pos[1] = raw.y * D2R;
    pos[2] = raw.z;

    ecef2enu(pos,r,e);
    rtk_e->x = e[0];
    rtk_e->y = e[1];
    rtk_e->z = e[2];
    //std::cout << "T1DecodeNMEA-pos2enu step3:" <</* list <<*/ e[0] << e[1] <<e[2];// << raw.lon << raw.alt;

}


void T1DecodeNMEA::pos2xyz(rtkPoint *rtx, std::vector<std::string> list)//QStringList list)
{
    double pos[3],r[3];
    nmea_raw_t2 raw;
    //std::cout << "T1DecodeNMEA-pos2xyz step0:" <<list;// <<*/ raw.lat << raw.lon << raw.alt;

    decode_nmeaGGA(&raw,list);
    //std::cout << "T1DecodeNMEA-pos2xyz step1:" <</* list <<*/ raw.lat << raw.lon << raw.alt;
    pos[0] = raw.lat;
    pos[1] = raw.lon;
    pos[2] = raw.alt;
    pos2ecef(pos,r);
    //std::cout << "T1DecodeNMEA-pos2xyz step2:" << pos[0] << pos[1] << pos[2] << r[0] << r[1] << r[2] << list;
    rtx->x = r[0];
    rtx->y = r[1];
    rtx->z = r[2];
    //std::cout << "T1DecodeNMEA-pos2xyz step3:" << rtx->x << rtx->y << rtx->z;// << r[0];

}

void T1DecodeNMEA::lla2xyz(rtkPoint *rtx, rtkPoint rtklla)
{
    double pos[3],r[3];
    nmea_raw_t2 raw;
    //std::cout << "T1DecodeNMEA-pos2xyz step0:" <<list;// <<*/ raw.lat << raw.lon << raw.alt;

    //decode_nmeaGGA(&raw,list);
    //std::cout << "T1DecodeNMEA-pos2xyz step1:" <</* list <<*/ raw.lat << raw.lon << raw.alt;
    pos[0] = rtklla.x * D2R;
    pos[1] = rtklla.y * D2R;
    pos[2] = rtklla.z;
    pos2ecef(pos,r);
    //std::cout << "T1DecodeNMEA-pos2xyz step2:" << pos[0] << pos[1] << pos[2] << r[0] << r[1] << r[2] << list;
    rtx->x = r[0];
    rtx->y = r[1];
    rtx->z = r[2];
    //std::cout << "T1DecodeNMEA-pos2xyz step3:" << rtx->x << rtx->y << rtx->z;// << r[0];

}

//double T1DecodeNMEA::
extern double dmm2deg(double dmm)
{
    float deg = floor(dmm/100.0)+fmod(dmm,100.0)/60.0;
    //std::cout << "T1DecodeNMEA-dmm2deg: " << dmm << deg;
    return deg;
}

//static const int nmea_solq[]={  /* NMEA GPS quality indicator [1] */
    /* 0=Fix not available or invalidi */
    /* 1=GPS SPS Mode, fix valid */
    /* 2=Differential GPS, SPS Mode, fix valid */
    /* 3=GPS PPS Mode, fix valid */
    /* 4=Real Time Kinematic. System used in RTK mode with fixed integers */
    /* 5=Float RTK. Satellite system used in RTK mode, floating integers */
    /* 6=Estimated (dead reckoning) Mode */
    /* 7=Manual Input Mode */
    /* 8=Simulation Mode */
//    SOLQ_NONE ,SOLQ_SINGLE, SOLQ_DGPS, SOLQ_PPP , SOLQ_FIX,
//    SOLQ_FLOAT,SOLQ_DR    , SOLQ_NONE, SOLQ_NONE, SOLQ_NONE
//};
/*
格式：
$GNGGA,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12>,<13>,<14>*<15><CR><LF>
<1> UTC时间，格式为hhmmss.sss
<2> 纬度，格式为ddmm.mmmm（前导位数不足则补0）
<3> 纬度半球，N或S（北纬或南纬）
<4> 经度，格式为dddmm.mmmm（前导位数不足则补0）
<5> 经度半球，E或W（东经或西经）
<6> 定位质量指示，0=定位无效，1=定位有效
<7> 使用卫星数量，从00到12（前导位数不足则补0）
<8> 水平精确度，0.5到99.9
<9> 天线离海平面的高度，-9999.9到9999.9米
<10> 高度单位，M表示单位米
<11> 大地椭球面相对海平面的高度（-999.9到9999.9）
<12> 高度单位，M表示单位米
<13> 差分GPS数据期限（RTCM SC-104），最后设立RTCM传送的秒数量
<14> 差分参考基站标号，从0000到1023（前导位数不足则补0）
<15> 校验和
*/
/* decode NMEA GGA (Global Positioning System Fix Data) sentence -------------*/
int T1DecodeNMEA::decode_nmeaGGA(nmea_raw_t2 *raw, std::vector<std::string> val)// QStringList val)
{
    volatile double lat=0.0,lon=0.0,alt=0.0,msl=0.0;
    //volatile double tod=0.0,hdop=0.0,age=0.0;
    std::string ns="N", ew="E", ua, um;
    int solq=0,nrcv=0;
    //std::cout << "T1DecodeNMEA-decode_nmeaGGA step1: " << val;

    for ( int i=1; i<val.size(); i++ ) {
        //int j = i-1;
        switch (i-1) {
            //case  0: tod = val[i].toFloat(); break; /* UTC of position (hhmmss) */
            case  1: lat = std::stof(val[i]); break; /* Latitude (ddmm.mmm) */
            case  2: ns  = val[i];      	 break; /* N=north,S=south */
            case  3: lon = std::stof(val[i]); break; /* Longitude (dddmm.mmm) */
            case  4: ew  = val[i];       	 break; /* E=east,W=west */
            case  5: solq= std::stoi(val[i]);	 break; /* GPS quality indicator */
            case  6: nrcv= std::stoi(val[i]);	 break; /* # of satellites in use */
            //case  7: hdop= val[i].toFloat(); break; /* HDOP */
            case  8: alt = std::stof(val[i]); break; /* Altitude MSL */
            case  9: ua  = val[i];      	 break; /* unit (M) */
            case 10: msl = std::stof(val[i]); break; /* Geoid separation */
            case 11: um  = val[i];      	 break; /* unit (M) */
            //case 12: age = val[i].toFloat(); break; /* Age of differential */
        }
        //std::cout << "T1DecodeNMEA-decode_nmeaGGA loop:" << i << val[i] << val[i].toFloat();
    }
    if ( (ns!="N"&&ns!="S") || (ew!="E"&&ew!="W") ) {
        //std::cout << "invalid nmea gga format\r\n";
        return 0;
    }
    //std::cout << "T1DecodeNMEA-decode_nmeaGGA step2: " << lat << dmm2deg(lat) << lon << dmm2deg(lon) << D2R;
    raw->lat=(ns=="N"?1.0:-1.0)*dmm2deg(lat)*D2R;
    raw->lon=(ew=="E"?1.0:-1.0)*dmm2deg(lon)*D2R;
    raw->alt=alt+msl;
    raw->nrcv = nrcv;
    raw->solq = solq; //nmea_solq[solq];
    return 1;
}
