#ifndef T1MAP2_HPP
#define T1MAP2_HPP

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <errno.h>
#include <sys/time.h>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>

#include "T1Map.hpp"

// value of each Grid map, >0 means the plan lines' number+1, <-1000 means obstacle found
#define VALUEGORBSTACLE (-1000)    // apriltag
#define VALUEGTAG (-500)    // apriltag
#define VALUEGFORB (-255)    // forbidden area
#define VALUEGBORD (-252)    // border unit
//#define VALUEGBORDIN (-251)    // grid beside the border unit's inside
#define VALUEGCUT (-250)    // unit that has been cut
#define VALUEGPASSONLY (-200)    //  unit that can pass passed by mower, but not planned to cut, such as the bridge between two area
#define VALUEGASTA (-100)    // A star path
#define VALUEGGRASS (0)    // grass unit

// Structure representing a node in the grid
struct GridNode {
    int x, y;               // Coordinates (maps to myGrid[x][y])
    int g;                  // Cost from start node to current node
    int h;                  // Heuristic estimate to end node
    GridNode* parent;           // Pointer to parent node (for path reconstruction)

    // Constructor
    GridNode(int x_, int y_, int g_ = 0, int h_ = 0, GridNode* p = nullptr)
        : x(x_), y(y_), g(g_), h(h_), parent(p) {}

    // Total cost function: f(n) = g(n) + h(n)
    int f() const { return g + h; }
};

// Comparator for priority queue (min-heap based on f value)
struct CompareNode {
    bool operator()(const GridNode* a, const GridNode* b) {
        return a->f() > b->f();  // Smallest f value is prioritized
    }
};

// result returned by AstarSearch
struct AStarResult {
    std::vector<std::pair<int, int>> path;  // the path from start to end
    int g_value = -1;                  // the step from start to end
    bool has_path;                // if has path

    // default think no path exist
    AStarResult() : g_value(0), has_path(false) {}
};

// Line segment structure: uses MapPoint for start and end points
struct LineSegment {
    MapPoint Start;  // Start point of the segment (MapPoint type)
    MapPoint End;    // End point of the segment (MapPoint type)
    // Constructor with MapPoint parameters
    //LineSegment(MapPoint s, MapPoint e) : start(s), end(e) {}
};

class T1Map2 : public T1Map {
public:
    std::vector<MapPoint> myRoutePlan2;  // route lines when scan grid map
    std::vector<LineSegment> myRoutePlanLine;  // route lines when scan grid map
    std::vector<MapPoint> myMapAprilTag;  // April Tag position set by user in map file

private:
    std::vector<MapPoint> myMapForbidden;  // forbidden area set by user in map file
    std::vector<MapPoint> myMapAreaPassOnly;  // area can pass set by user in map file
    //std::vector<LineSegment> segmentQueue;
    //std::vector<MapPoint> myRoutePlan2;  // route lines when scan grid map
    //std::vector<LineSegment> myRoutePlanLine;  // route lines when scan grid map

    int meWidthCutter = 30;
    int meWidthGrid = 20;
    int meAnglePlan = 10;

    int meWidthBorder = 0;
    int meGridMaxX = 0;
    int meGridMaxY = 0;

private:
    int GetGrid();
    void SetGridOutside();
    void SetGridBorder();
    void SetGridBorderInside();
    //void SetGridInside(int,int);
    void SetAreaForbidden();
    void SetAreaPassOnly();
    void SetPassOnlyCircle(int,int,int);
    void SetPassOnlyRect(int,int,int,int,int);
    void SetForbiddenCircle(int,int,int);
    void SetForbiddenRect(int,int,int,int,int);
    void ReadForbidden(MapPoint) override;
    void ReadAreaPassOnly(MapPoint) override;
    void ReadAprilTagPos(MapPoint) override;
    int GetRemote() override;
    //void printRouteLines();
    void SetGridLine(int,int,int,int,int);
    void SetGridLineRoute();
    void GetRouteLines(int);
    void GetRouteLinesSingle(int, double, double, double, double, int, int, LineSegment&);
    int GetRoute2(int); 
    bool isGridAccessible(int, int);
    
    int AstarManhattan(const GridNode* node, const GridNode& end);
    bool AstarIsValid(int x, int y);
    AStarResult AstarSearch(int start_x, int start_y, int end_x, int end_y);
    std::vector<std::pair<int, int>> AstarSimplifyPath(const std::vector<std::pair<int, int>>& original_path);
    int findA();
    //void findBorderPointClear();
    void findBorderPointInside(int *, int *);

public:
    T1Map2();
    ~T1Map2();
    void Init() override;
    int GetRoute(int) override; 
    int GetLength(char) override;
    virtual int SetAprilTagChrgStation(int) override;
    int GetRoute3(int); 
    int GetRouteBorder();
    MapPoint GetPoint2(char, int);
    void SetValue(char, int);
    int findNearestLine(int, int);
    int findNearestLineBY(int, int);
    int findAstarNear(int, int, int, int);
    void SetGridLineCut(int, int, int, int);
    int findAstarToPoint(int, int, int, int);  // Calculate A* path from (currX, currY) to (targetX, targetY)
    //unsigned char GridValueTo2Bit(int gridValue);
    //void Save2BitMapToFile(const std::string& filename); 
    void GridMapSave(const char *);
    void GridMapRead(const char *);
    void SetGridObstacle(float, float, float, float);

    int **myGrid = NULL;
    int *dataGrid = NULL;

private:
    //int RouteALat();
};

#endif // T1MAP2_HPP

