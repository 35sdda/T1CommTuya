#include <iostream>
#include <cstdio>
#include <chrono>
#include <thread>

#include <fstream>
#include <string>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <algorithm>


#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <cstring>
#include <regex>   
#include <stdexcept>
#include <unistd.h>
#include <stdio.h>
#include <time.h>
//#include <sys/time.h>
#include <errno.h>
#include <ctype.h>
#include <fcntl.h>

#include "T1Serial.hpp"
#include "T1Socket.hpp"
#include "T1BlueT.hpp"
#include "T1All.hpp"

#define FILE_BUFFER_SIZE 4096 

uint64_t ntohll(uint64_t net_val);
std::string readCmdStr();
std::string tranCmdStr(T1All *, char type, int key);
bool FileRecvFromApp(T1Socket *, const std::string&, std::string& received_filename);
//typedef std::chrono::high_resolution_clock::time_point HRTimePoint;
int timeCalculate(int, uint64_t *, uint64_t *, const char *);

int isTimeSetOk = 1;

