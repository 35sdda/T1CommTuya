#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <errno.h>
#include <sys/time.h>
#include <vector>
#include "T1Act.hpp"

class T1ActSimu : public T1Act {
protected:
    int WorkLine(MapPoint) override;
    int Work() override;
    
public:
    T1ActSimu();
    ~T1ActSimu() ;
    
    int WorkAll() override;

private:

};

