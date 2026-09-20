#include "T1Utils.hpp"
#include <fstream>
#include <cstdio>
#include <iostream>
#include <regex>
#include <unistd.h>
#include <stdexcept>

namespace T1Utils {

// Member Variables Definition
std::string meSsid;

// Member Functions Implementation

/**
 * Copy file
 * @param srcFile Source file path
 * @param dstFile Destination file path
 * @return true=success, false=failure
 */
bool copyFile(const char* srcFile, const char* dstFile) {
    std::ifstream fin(srcFile, std::ios::binary);
    std::ofstream fout(dstFile, std::ios::binary | std::ios::trunc);
    
    if (!fin) {
        printf("ERROR: T1Utils-copyFile() cannot open source file: %s\n", srcFile);
        return false;
    }
    
    if (!fout) {
        printf("ERROR: T1Utils-copyFile() cannot create destination file: %s\n", dstFile);
        return false;
    }
    
    // Copy file content
    fout << fin.rdbuf();
    
    // Close files to ensure data is written to disk
    fin.close();
    fout.close();
    
    printf("T1Utils-copyFile() success: %s -> %s\n", srcFile, dstFile);
    return true;
}

/** Log to custom log files with custom filename
 * 2026.05.07: created by Xs
 * 2026.05.15: refactored to use T1Utils time functions
 * 2026.05.18: modified to add printf control parameter
 * @param data Log data to write
 * @param filename Custom log filename (without path, will be saved to ../Log/)
 * @param printfFlag 0=no printf, 1=print to console with printf
 */
void LogRecord(const char * data, const char *filename, int printfFlag) {
     char fullpath[200], timeStr[30];

     // Print to console if printfFlag is 1
     if (printfFlag == 1) {
         printf("%s\n", data);
     }

     // Get current date and time using unified T1Utils functions
     std::string mydate = T1Utils::getCurrentYYYYMMDD();
     std::string hhmmss = T1Utils::getCurrentHHMMSS();
     uint64_t timeUs = T1Utils::getCurrentTimeUs();

     // Format time string with microseconds: [HH:MM:SS.ffffff]
     snprintf(timeStr, sizeof(timeStr), "[%s.%06ld]", hhmmss.c_str(), timeUs % 1000000);

     int ret = snprintf(fullpath, sizeof(fullpath), "../Log/%s%s.log",
                        filename, mydate.c_str());
     if ( ret<0 || ret>=(int)sizeof(fullpath) ) {
         std::cerr << "T1Utils-LogRecord, filename too long" << std::endl;
         return;
     }

     std::ofstream fileout(fullpath, std::ios::app);
     if (!fileout.is_open()) {
         std::cerr << "T1Utils-LogRecord, Can not open log file:" << fullpath << std::endl;
         return;
     }
     fileout << timeStr << "[" << data << "]" << std::endl;
     fileout.close();
}
/** Log Bugs to log files
 * 2026.02.27: created by Leo
 * 2026.05.15: refactored to use T1Utils time functions
 */
void LogBug(const char * data) {
    int debug=1;
    char filename[80], timeStr[30];
    
    if (debug>0) std::cerr << data << std::endl;

    // Get current date and time using unified T1Utils functions
    std::string mydate = T1Utils::getCurrentYYYYMMDD();
    std::string hhmmss = T1Utils::getCurrentHHMMSS();
    uint64_t timeUs = T1Utils::getCurrentTimeUs();
    
    // Format time string with microseconds: [HH:MM:SS.ffffff]
    snprintf(timeStr, sizeof(timeStr), "[%s.%06ld]", hhmmss.c_str(), timeUs % 1000000);

    std::string ssid = T1Utils::meSsid;
    int ret = snprintf(filename, sizeof(filename), "../Log/%sBug%s.log", ssid.c_str(), mydate.c_str());
    if ( ret<0 || ret>=(int)sizeof(filename) ) {
        std::cerr << "T1Utils-LogBug, filename too long" << std::endl;
        return;
    }

    std::ofstream fileout(filename, std::ios::app);
    if (!fileout.is_open()) {
        std::cerr << "T1Utils-LogBug, Can not open log file:" << filename << std::endl;
        return;
    }
    fileout << timeStr << "[" << data << "]" << std::endl;
    fileout.close();
}

/** split string into parts seperate by 'ch'
 * 2026.02.27: created by Leo
 */
std::vector<std::string> split(const std::string& str, char ch) {
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

/**
 * Read SSID from hostapd.conf
 * 2026.03.16: created by BY
 */
std::string read_hostapd_ssid() {
    std::string hostapd_path = "/etc/hostapd.conf";
    // Check if file exists and is readable (root-owned files require proper permissions)
    if (access(hostapd_path.c_str(), R_OK) != 0) {
        throw std::runtime_error("Failed to access hostapd.conf file: " + hostapd_path + " (insufficient permissions or file does not exist)");
    }

    std::ifstream file(hostapd_path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open hostapd.conf: " + hostapd_path);
    }

    // Regex match: ssid=xxx (ignore leading whitespace, comments, capture all chars after equals sign)
    const std::regex ssid_regex(R"(^\s*ssid\s*=\s*(.+)\s*$)", std::regex::icase);
    std::string line;
    std::string ssid;

    while (std::getline(file, line)) {
        // Skip comment lines (starting with #) and empty lines
        if ( line.empty() || line.find_first_of("#")==0 ) {
            continue;
        }

        std::smatch match;
        if (std::regex_match(line, match, ssid_regex)) {
            ssid = match[1].str();
            break; // Exit after first valid SSID (avoid duplicate configs)
        }
    }

    file.close();

    if (ssid.empty()) {
        throw std::runtime_error("T1Utils-read_hostapd_ssid(), No valid SSID configuration found in hostapd.conf");
    } else {
        printf("T1Utils-read_hostapd_ssid() success, SSID: %s\n", ssid.c_str());
    }

    return ssid;
}

/**
 * Get SSID and store to meSsid
 * 2026.03.17: created by BY
 */
void get_hostapd_ssid() {
    meSsid = read_hostapd_ssid();
}
/**Get current time in YYYYMMDD format since epoch
 * 2026.05.14: created for unified time API
 */
std::string getCurrentYYYYMMDD() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    char buffer[11];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d", std::localtime(&time_t));
    return std::string(buffer);
}

/**Get current time in HHMMSS format since epoch
 * 2026.05.14: created for unified time API
 */
std::string getCurrentHHMMSS() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    char buffer[9];
    std::strftime(buffer, sizeof(buffer), "%H:%M:%S", std::localtime(&time_t));
    return std::string(buffer);
}

/**Get current time in milliseconds since epoch
 * 2026.05.14: created for unified time API
 */
uint64_t getCurrentTimeMs() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()
    ).count();
}

/**Get current time in microseconds since epoch
 * 2026.05.14: created for unified time API
 */
uint64_t getCurrentTimeUs() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()
    ).count();
}


/** transfer Longitude and Latitude's format
 * 2026.01.20: created by Leo, moved form T1Act to T1Map
 * 2026.05.19: moved form T1Map to T1Utils by Leo
 */
double TransLonLat(const double val) {
    double tmpval, retval;
    tmpval = (int)val / 100;
    retval = ( val - tmpval*100.0 ) / 60.0 + tmpval;
    return retval;
}


} // namespace T1Utils
