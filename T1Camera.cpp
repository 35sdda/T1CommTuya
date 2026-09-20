#include "T1Camera.hpp"
//#include <sys/time.h>
//#include <time.h>
//#include <iostream>
//#include <fstream>

T1Camera::T1Camera()
{

}

T1Camera::~T1Camera()
{
}

std::vector<std::string> T1Camera::split(const std::string& str, char ch) {
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
