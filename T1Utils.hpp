#ifndef T1UTILS_HPP
#define T1UTILS_HPP

#include <sys/time.h>
#include <ctime>
#include <vector>
#include <string>
#include <chrono>
#include <cstdint>

/**
 * T1项目通用工具函数库
 * T1 Project Common Utility Functions
 * 
 * 职责：提供项目中常用的工具函数，与业务逻辑解耦
 * Responsibility: Provide common utility functions, decoupled from business logic
 */

namespace T1Utils {

// Member Variables
extern std::string meSsid;

// Member Functions

/**
 * Copy file
 * @param srcFile Source file path
 * @param dstFile Destination file path
 * @return true=success, false=failure
 */
bool copyFile(const char* srcFile, const char* dstFile);

/**
 * Split string
 * @param str String to split
 * @param ch Delimiter character
 * @return List of split strings
 */
std::vector<std::string> split(const std::string& str, char ch);
void LogRecord(const char *, const char *,int printfFlag=0);
void LogBug(const char *);
/**
 * Read SSID from hostapd.conf
 * @return SSID string
 * @throws runtime_error if read fails
 */
std::string read_hostapd_ssid();
/**
 * Transfer Longitude and Latitude's format
 * @param val Value in format like "ddmm.mmmm" (degrees and minutes)
 * @return Value in decimal degrees format
 */
double TransLonLat(const double val);
/**
 * Get SSID and store to meSsid
 */
void get_hostapd_ssid();

/**
 * Get current time in milliseconds since epoch
 * @return Current time as uint64_t (milliseconds)
 */
uint64_t getCurrentTimeMs();

/**
 * Get current time in microseconds since epoch
 * @return Current time as uint64_t (microseconds)
 */
uint64_t getCurrentTimeUs();

/**
 * Get current date in YYYYMMDD format
 * @return Date string in "YYYYMMDD" format
 */
std::string getCurrentYYYYMMDD();

/**
 * Get current time in HHMMSS format
 * @return Time string in "HHMMSS" format
 */
std::string getCurrentHHMMSS();

// Future extensions:
// bool deleteFile(const char* file);
// bool fileExists(const char* file);
// std::string getFileExtension(const char* file);
// ...

} // namespace T1Utils

// App time functions (defined in TheOne.cpp)
// Get current time using App time (if available) or system time
extern time_t getAppTime();

// Get current time with microseconds using App time
extern void getAppTimeOfDay(struct timeval *tv);

#endif // T1UTILS_HPP
