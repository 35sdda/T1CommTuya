#ifndef MYDECODENMEA_H
#define MYDECODENMEA_H
#include <vector>
#include <string>
//#include <cstring>
//#include <QTextStream>


/* NMEA parse */
#define MAXRAWLEN   (128)

/* solution quality */
#define SOLQ_NONE   0                   /* solution status: no solution */
#define SOLQ_FIX    1                   /* solution status: fix */
#define SOLQ_FLOAT  2                   /* solution status: float */
#define SOLQ_SBAS   3                   /* solution status: SBAS */
#define SOLQ_DGPS   4                   /* solution status: DGPS/DGNSS */
#define SOLQ_SINGLE 5                   /* solution status: single */
#define SOLQ_PPP    6                   /* solution status: PPP */
#define SOLQ_DR     7                   /* solution status: dead reconing */

struct rtkPoint {
    double x=0.0;
    double y=0.0;
    double z=0.0;
};

typedef struct
{
    int         nbyte;                  /* number of bytes in message buffer */
    int         len;
    int         type;                   /* which type of nmea sentense */
    char        buf[MAXRAWLEN];         /* message raw buffer */
    double      lat,lon,alt;            /* lat: in RAD, lon: in RAD, alt:m */
    int         solq;                   /* solq: solution quality */
    int         nrcv;                   /* nrcv: num of sat used */
    char        nmea_hdr[3];            /* "GGA", "RMC", etc */
} nmea_raw_t2;


class T1DecodeNMEA
{
public:
    //MyDecodeNMEA();

    void lla2xyz(rtkPoint *, rtkPoint);
    void lla2enu(rtkPoint *, rtkPoint, rtkPoint);

    void pos2xyz(rtkPoint *, std::vector<std::string>);//QStringList);
    void pos2enu(rtkPoint *, rtkPoint, std::vector<std::string>);//QStringList);

private:
    int decode_nmeaGGA(nmea_raw_t2 *, std::vector<std::string>);//QStringList);
};

#endif // MYDECODENMEA_H
