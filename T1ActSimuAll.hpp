#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <errno.h>
#include <sys/time.h>
#include <vector>
#include "T1ActSimu.hpp"

class T1ActSimuAll : public T1ActSimu {
private:
    int SetObst(Wall);
    void SetObstSimu(int);
    void SetCutLine(MapPoint, MapPoint);
    void SetCutGrid();
    void GoHomeGridMapSave();
    void GoHomeGridMapRead();
    int GoHomeSavePoint();
    int GoHomeReadPoint();
    int GoHomeClearPoint();
    int GoHome();
    int GoBorder();
    //int WorkLine2(MapPoint,MapPoint);
    //int WorkLine(MapPoint) override;
    //int Work() override;
    int Work2() override;
    
public:
    T1ActSimuAll();
    ~T1ActSimuAll();
    
    int WorkAll() override;
    PointRobo Cleft, Cright;

private:
    int meWorkline2 = 0;
    int meWorklength2 = 0;
    int flagReverse = 0; // direction of cut line, 0 means error, >0 means normal direction, <0 means reverse direction
    PointRobo GoHomePoint;

};

