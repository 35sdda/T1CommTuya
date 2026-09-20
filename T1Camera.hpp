#ifndef T1Camera_H
#define T1Camera_H

#include <vector>
#include <stdio.h>
#include <string>

class T1Camera
{
private:

public:
    T1Camera();
    ~T1Camera();
    
private:
    std::vector<std::string> split(const std::string& str, char ch);

};


#endif // T1Camera_H
