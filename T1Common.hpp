#ifndef THEONECOMMON_HPP
#define THEONECOMMON_HPP

#include <cstdint>

typedef struct PointRobo { // this is used for visual position and robot's position
    float fx=0.0;
    float fy=0.0;
    int yaw=0;
    int roll=0;
    int pitch=0;
    float gyroX=0.0;
    float gyroY=0.0;
    float gyroZ=0.0;
    float accX=0.0;
    float accY=0.0;
    float accZ=0.0;
    uint64_t timeIMU=0;
    uint32_t timeIMU415=0;
    uint64_t timeSet = 0;
    // the mower's position set by WheelOdometry/AprilTag/VIO, W/A/O, default is '='
    char flagPosSetBy = '=';
} PointRobo;

// Full AprilTag pose. All translations are in centimeters.
// rotation is row-major and transforms the corrected Tag frame to camera frame.
typedef struct AprilTagPose {
    int tagId = -1000;
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    double rotation[9] = {0.0};
    double relativeLateral = 0.0;      // Signed lateral offset from the Tag center.
    double relativeVertical = 0.0;     // Vertical offset in the Tag frame.
    double normalDistance = 0.0;       // Distance along the Tag surface normal.
    double yawError = 0.0;             // Signed heading error relative to the Tag.
    double reprojectionErrorPx = 0.0;  // Final corner reprojection RMS in pixels.
    uint64_t timeSet = 0;
    bool valid = false;
} AprilTagPose;

class MapPoint { // this is used for RTK/IMU/Map position
public:
    long long lon=0;
    long long lat=0;
    int alt=0;
    int roll=0;
    int pitch=0;
    int yaw=0;
    float gyroX=0.0;
    float gyroY=0.0;
    float gyroZ=0.0;
    float accX=0.0;
    float accY=0.0;
    float accZ=0.0;
    uint64_t timeIMU=0;
    uint32_t timeIMU415=0;
    int fixed=0;
    int sate=0;
    uint64_t timeSet = 0;
    double batteryVoltage=0.0;//Battery voltage
};

typedef struct Wall {
    float leftDepth;   // Depth of left edge
    float leftOffset;  // Horizontal offset from image center (left edge)
    float rightDepth;  // Depth of right edge
    float rightOffset; // Horizontal offset from image center (right edge)
} Wall;

// STA status data from 415 firmware
// Format: $STA,ver,bat,cflag,L,R,C,RL,RR,S1,S2,Rain,gHome
typedef struct STAStatus {
    char version[16] = "0.0.0";      // Firmware version, e.g., "0.3.1"
    double batteryVoltage = 0.0; // Battery voltage, e.g., 12.45
    int chargeFlag = 0;          // Charging marker: 0=not charging, 1=charging
    int senFrontLeft = 0;        // Sensor front left (L)
    int senFrontRight = 0;       // Sensor front right (R)
    int senCenter = 0;           // Sensor center (C)
    int senRearLeft = 0;         // Sensor rear left (RL)
    int senRearRight = 0;        // Sensor rear right (RR)
    int senStop1 = 1;            // Sensor stop1 (S1)
    int senStop2 = 1;            // Sensor stop2 (S2)
    int senRain = 1;             // Sensor rain (Rain): 0=raining, 1=no rain
    int goHome = 0;              // Go home flag: 0=no, 1=yes
    uint64_t timeSet = 0;        // Timestamp when data was set
} STAStatus;

#define LENGTHSTRSEND 200
#define PRECISION (10000000.0)

#endif // THEONECOMMON_HPP
