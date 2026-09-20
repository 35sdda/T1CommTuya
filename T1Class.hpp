#ifndef T1Class_H
#define T1Class_H

#include <vector>
#include <stdio.h>
#include <string>

class T1Class
{
private:

public:
    T1Class();
    ~T1Class();
    
private:
    std::vector<std::string> split(const std::string& str, char ch);

};


#endif // T1Class_H
