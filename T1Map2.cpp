#include "T1Map2.hpp"
#include "T1Utils.hpp"
#include <vector>
#include <string>
#include <cstring>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <math.h>


T1Map2::T1Map2() {

}

T1Map2::~T1Map2() {
    free(myGrid);
    myGrid = NULL;
    free(dataGrid);
    dataGrid = NULL;
}

void T1Map2::Init() {
    printf("T1Map2-Init begin.  do nothing\n");
    GetRoute(meTypeRoutePlan);
    printf("T1Map2-Init end. myMapForbidden=%ld, myMapAprilTag=%ld\n", myMapForbidden.size(), myMapAprilTag.size());
}

// return the size of the grid map
int T1Map2::GetLength(char type) {
    switch (type) {
    case 'B':
        return myMapBorder.size();
        break;
    case 'R':
        return myMapRemote.size();
        break;
    case 'P':
        return myRoutePlan.size();
        break;
    case 'L':
        return myRoutePlanLine.size();
        break;
    case '2':
        return myRoutePlan2.size();
        break;
    case 'x':
        return meGridMaxX;
        break;
    case 'y':
        return meGridMaxY;
        break;
    case 'c':
        return meWidthCutter;
        break;
    case 'g':
        return meWidthGrid;
        break;
    default:
        printf("T1Map2-GetLength, unknown type[%c]\n", type);
        return -1;
        break;
    }
}

// set the Cutted grid to 251
// void T1Map2::SetGridLineCut(int x1, int y1, int x2, int y2) {
//     int gx1,gx2,gy1,gy2;
//     gx1 = ( x1-mapMin.lon ) / meWidthGrid + 1;
//     gy1 = ( y1-mapMin.lat ) / meWidthGrid + 1;
//     gx2 = ( x2-mapMin.lon ) / meWidthGrid + 1;
//     gy2 = ( y2-mapMin.lat ) / meWidthGrid + 1;
//     SetGridLine(gx1,gy1,gx2,gy2,VALUEGCUT);
// }

/**
 * Reduce integer truncation error
 * 2026.04.16: created by Xs 
 * 2026.05.24：modified by BY
 * */
void T1Map2::SetGridLineCut(int x1, int y1, int x2, int y2) {
    int gx1, gx2, gy1, gy2;
    gx1 = (x1 - mapMin.lon) / meWidthGrid + 1;
    gy1 = (y1 - mapMin.lat) / meWidthGrid + 1;
    gx2 = (x2 - mapMin.lon) / meWidthGrid + 1;
    gy2 = (y2 - mapMin.lat) / meWidthGrid + 1;
    SetGridLine(gx1, gy1, gx2, gy2, VALUEGCUT);
}

// set the RouteLines to the lines' number+1
void T1Map2::SetGridLineRoute() {
    int count = GetLength('L');
    for ( int i=0; i<count; i++ ) {
        int x1 = ( myRoutePlanLine[i].Start.lon-mapMin.lon ) / meWidthGrid + 1;
        int y1 = ( myRoutePlanLine[i].Start.lat-mapMin.lat ) / meWidthGrid + 1;
        int x2 = ( myRoutePlanLine[i].End.lon-mapMin.lon ) / meWidthGrid + 1;
        int y2 = ( myRoutePlanLine[i].End.lat-mapMin.lat ) / meWidthGrid + 1;
        //printf("T1Map2-SetGridLineRoute %d, (%d,%d), (%d,%d)\n", i, x1,y1, x2,y2);
        SetGridLine(x1,y1,x2,y2,i+1);
    }        
}

/** read the file to restore the gridmap
 * @brief Loads data from grid.dat into a 100x100 int 2D array
 * Validates file integrity and dimension matching before reading
 * @param grid The 2D int array to receive loaded data (pre-allocated)
 * @throw runtime_error Throws exception for file errors/dimension mismatch
 * 2026.02.28: created by Leo
 */
void T1Map2::GridMapRead(const char *filename) {
    //printf("T1Map2-GridMapRead begin.[%s]\n", filename);
    // Step 1:  Open file in binary read mode
    std::ifstream in_file(filename, std::ios::binary | std::ios::in);
    if (!in_file.is_open()) {
        throw std::runtime_error("Failed to open file for reading: " + std::string(filename));
    }
    // Step 2: Read grid data into memory (continuous block for efficiency)
    //in_file.read(reinterpret_cast<char*>(myGrid), meGridMaxX * meGridMaxY * sizeof(int));
    for (int i = 0; i < meGridMaxX; ++i) {
        in_file.read(reinterpret_cast<char*>(myGrid[i]), meGridMaxY * sizeof(int));
        // Check read error for current row
        if (!in_file) {
            in_file.close();
            throw std::runtime_error("Failed to read row " + std::to_string(i) + " from file");
        }
    }
    // Check if read operation completed successfully
    if (!in_file) {
        in_file.close();
        throw std::runtime_error("Failed to read data from file: " + std::string(filename));
    }
    // Close file stream
    in_file.close();
    for ( int i=0; i<meGridMaxX; i++ ) {
        for ( int j=0; j<meGridMaxY; j++ ) {
            //printf("%d,%d,",i,j); //
            //printf("%d\n", myGrid[i][j]);
        }
    }
    printf("T1Map2-GridMapRead end. read[%s]\n", filename);
}

/** save the current gridmap before gohome()
 * @brief Saves a 100x100 int 2D array to grid.dat in binary format
 * Binary format is preferred for embedded systems (faster, smaller file size)
 * @param grid The 2D int array to be saved
 * @throw runtime_error Throws exception if file operation fails (for upper-layer handling)
 * 2026.02.28: created by Leo
 */
void T1Map2::GridMapSave(const char *filename) {
    //printf("T1Map2-GridMapSave begin.[%s]\n", filename);
    // Step 1: Open file in binary write mode (ios::trunc overwrites existing file)
    std::ofstream out_file(filename, std::ios::binary | std::ios::out | std::ios::trunc);
    if (!out_file.is_open()) {
        throw std::runtime_error("Failed to open file for writing: " + std::string(filename));
    }
    // Step 2: Write entire grid data (2D array as continuous memory block)
    //out_file.write(reinterpret_cast<const char*>(myGrid), meGridMaxX * meGridMaxY * sizeof(int));
    for (int i = 0; i < meGridMaxX; ++i) {
        out_file.write(reinterpret_cast<const char*>(myGrid[i]), meGridMaxY * sizeof(int));
        // Check write error for current row
        if (!out_file) {
            out_file.close();
            throw std::runtime_error("Failed to write row " + std::to_string(i) + " to file");
        }
    }
    // Check if write operation completed successfully
    if (!out_file) {
        out_file.close();
        throw std::runtime_error("Failed to write data to file: " + std::string(filename));
    }
    // Close file stream (critical for embedded systems to avoid resource leaks)
    out_file.close();
    printf("T1Map2-GridMapSave end. save[%s]\n", filename);
}

/* Set grids along a line segment to obstacle color(number),
 * The obstacle value will decrease over time.
 * 2026.05.20: created by Leo
 */
void T1Map2::SetGridObstacle(float fx1, float fy1, float fx2, float fy2) {
    // Extension length for both ends of the line
    float exlen = meWidthCutter / 2 + meWidthGrid;
    // Extend the line segment by exlen on BOTH ends (only if points are different)
    float dx = fx2 - fx1;
    float dy = fy2 - fy1;
    float len = sqrt(dx * dx + dy * dy);

    // Avoid division by zero for very short lines, if points are the same, no extension add is a bug
    if (len > 0.0001f) {
        fx1 -= (dx / len) * exlen;
        fy1 -= (dy / len) * exlen;
        fx2 += (dx / len) * exlen;
        fy2 += (dy / len) * exlen;
    }
    // Coordinate conversion 
    int x1 = (fx1 - mapMin.lon) / meWidthGrid + 1;
    int y1 = (fy1 - mapMin.lat) / meWidthGrid + 1;
    int x2 = (fx2 - mapMin.lon) / meWidthGrid + 1;
    int y2 = (fy2 - mapMin.lat) / meWidthGrid + 1;
    // Bresenham line algorithm initialization
    int dx_grid = abs(x2 - x1);
    int dy_grid = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx_grid - dy_grid;

    printf("T1Map2-SetObstacle begin, Max(%d,%d), xy1(%d,%d), xy2(%d,%d)\n", meGridMaxX, meGridMaxY, x1, y1, x2, y2);
    while (true) {
        // Set obstacle if current grid is valid
        if (x1 >= 0 && x1 < meGridMaxX && y1 >= 0 && y1 < meGridMaxY) {
            if (myGrid[x1][y1] >= VALUEGGRASS || myGrid[x1][y1] == VALUEGCUT ) {
            //if (myGrid[x1][y1] >= VALUEGGRASS || myGrid[x1][y1] == VALUEGCUT ) || myGrid[x1][y1] == VALUEGBORD) {
                myGrid[x1][y1] = VALUEGORBSTACLE;
            }
        }
        // Break loop when reaching the end point
        if (x1 == x2 && y1 == y2) {
            break;
        }
        // 4-directional movement (fixed, no infinite loop)
        int e2 = 2 * err;
        bool moved = false;
        if (e2 > -dy_grid && !moved) {
            err -= dy_grid;
            x1 += sx;
            moved = true;
        }
        if (e2 < dx_grid && !moved) {
            err += dx_grid;
            y1 += sy;
            moved = true;
        }
    }
    printf("T1Map2-SetObstacle end, Max(%d,%d), xy1(%d,%d), xy2(%d,%d)\n", meGridMaxX, meGridMaxY, x1, y1, x2, y2);
}

/* set the grid of obstacle to certain color(number), and this number will reduce on seconds
 * 2026.05.20: created by Leo
 */
/*void T1Map2::SetGridObstacle(float fx1, float fy1, float fx2, float fy2) {
    int x1 = ( fx1-mapMin.lon ) / meWidthGrid + 1;
    int y1 = ( fy1-mapMin.lat ) / meWidthGrid + 1;
    int x2 = ( fx2-mapMin.lon ) / meWidthGrid + 1;
    int y2 = ( fy2-mapMin.lat ) / meWidthGrid + 1;

    int dx = abs(x2-x1);
    int dy = abs(y2-y1);
    int sx = (x1<x2) ? 1 : -1;  // step direction for x
    int sy = (y1<y2) ? 1 : -1;  // step direction for y
    int err = dx-dy;
    printf("T1Map2-SetObstacle begin, Max(%d,%d), xy1(%d,%d), xy2(%d,%d)\n", meGridMaxX, meGridMaxY, x1,y1, x2,y2);

    while (true) {
    //printf("T1Map2-SetObstacle loop, Max(%d,%d), xy1(%d,%d), xy2(%d,%d)\n", meGridMaxX, meGridMaxY, x1,y1, x2,y2);
        if ( x1>=0 && x1<meGridMaxX && y1>=0 && y1<meGridMaxY ) {
            if ( myGrid[x1][y1]>=VALUEGGRASS || myGrid[x1][y1]==VALUEGBORD || myGrid[x1][y1]==VALUEGCUT )
            // 0 or planned cut lines can be set, cut and border also can be set
                myGrid[x1][y1] = VALUEGORBSTACLE;
        }
        if ( x1==x2 && y1==y2 ) break;

        // 4 directions continue, Prioritize moving along the X-axis first, then Y-axis
        int e2 = 2*err; // double the error term to acoid floating-point ops
        bool moved = false; // Key logic: Only move ONE axis at a time (ensure 4-neighborhood continuity)
        // 1. Try to move x axis first if error allows (closer to line direction)
        if (e2 > -dy && !moved) {
            err -= dy;
            x1 += sx;  // Move only x (left/right)
            moved = true;
        }
        // 2. If x is not moved, move y axis (ensure step is single-axis)
        if (e2 < dx && !moved) {
            err += dx;
            y1 += sy;  // Move only y (up/down)
            moved = true;
        }
        /* 8 directions continue
        int e2 = 2*err; // double the error term to acoid floating-point ops
        if ( e2 > -dy ) { // adjust error term and step in x direction if needed
            err -= dy;
            x1 += sx;   // move one unit in x direction
        }
        if ( e2 < dx ) { // adjust error term and step in y direction if needed
            err += dx;
            y1 += sy;   // move one unit in y direction
        }/*
    }
    printf("T1Map2-SetObstacle end, Max(%d,%d), xy1(%d,%d), xy2(%d,%d)\n", meGridMaxX, meGridMaxY, x1,y1, x2,y2);
}*/

// set the grid line to certain color(number)
void T1Map2::SetGridLine(int x1, int y1, int x2, int y2, int color) {
    //if ( x1<0 || x1>meGridMaxX-1 ) return;
    //if ( x2<0 || x2>meGridMaxX-1 ) return;
    //if ( y1<0 || y1>meGridMaxY-1 ) return;
    //if ( y2<0 || y2>meGridMaxY-1 ) return;
    //printf("T1Map2-SetGridLine begin, Max(%d,%d), xy1(%d,%d), xy2(%d,%d)\n", meGridMaxX, meGridMaxY, x1,y1, x2,y2);
    // fill the grid map with Bresenham's line algorithm
    int dx = abs(x2-x1);
    int dy = abs(y2-y1);
    int sx = (x1<x2) ? 1 : -1;  // step direction for x
    int sy = (y1<y2) ? 1 : -1;  // step direction for y
    int err = dx-dy;
    
    while (true) {
    //printf("T1Map2-SetGridLine loop, Max(%d,%d), xy1(%d,%d), xy2(%d,%d)\n", meGridMaxX, meGridMaxY, x1,y1, x2,y2);
        if ( x1>=0 && x1<meGridMaxX && y1>=0 && y1<meGridMaxY ) {
            //if ( myGrid[x1][y1] > VALUEGBORD ) // forbidden and borrder should not be set
            if ( myGrid[x1][y1] > VALUEGPASSONLY ) // forbidden and borrder should not be set
                myGrid[x1][y1] = color;
        }
        if ( x1==x2 && y1==y2 ) break;

        // 4 directions continue, Prioritize moving along the X-axis first, then Y-axis
        int e2 = 2*err; // double the error term to acoid floating-point ops
        bool moved = false; // Key logic: Only move ONE axis at a time (ensure 4-neighborhood continuity)
        // 1. Try to move x axis first if error allows (closer to line direction)
        if (e2 > -dy && !moved) {
            err -= dy;
            x1 += sx;  // Move only x (left/right)
            moved = true;
        }
        // 2. If x is not moved, move y axis (ensure step is single-axis)
        if (e2 < dx && !moved) {
            err += dx;
            y1 += sy;  // Move only y (up/down)
            moved = true;
        }
        /* 8 directions continue
        int e2 = 2*err; // double the error term to acoid floating-point ops
        if ( e2 > -dy ) { // adjust error term and step in x direction if needed
            err -= dy;
            x1 += sx;   // move one unit in x direction
        }
        if ( e2 < dx ) { // adjust error term and step in y direction if needed
            err += dx;
            y1 += sy;   // move one unit in y direction
        }*/
    }
}

/** clear myMapForbidden and myMapAprilTag and reuse T1Map::GetRemote()
 * 2026.05.01: creat by Leo
 */
int T1Map2::GetRemote() {
    //printf("T1Map2-GetRemote begin, Forbidden=%ld, AprilTag=%ld\n", myMapForbidden.size(), myMapAprilTag.size());
    myMapAreaPassOnly.clear();
    myMapForbidden.clear();
    myMapAprilTag.clear();
    //printf("T1Map2-GetRemote step1, cleared 2 vectors, Forbidden=%ld, AprilTag=%ld\n", myMapForbidden.size(), myMapAprilTag.size());
    int ret = T1Map::GetRemote();
    printf("T1Map2-GetRemote end, AreaPassOnly=%ld, Forbidden=%ld, AprilTag=%ld\n", myMapAreaPassOnly.size(), myMapForbidden.size(), myMapAprilTag.size());
    return ret;
}

// read the map's forbidden line and push into the vector
void T1Map2::ReadForbidden(MapPoint point) {
    // if rectangle, should judge x1,y1 musk less than x2,y2
    myMapForbidden.emplace_back(point);
}

// Read AreaPass vectors
void T1Map2::ReadAreaPassOnly(MapPoint point) {
    // if rectangle, should judge x1,y1 musk less than x2,y2
    myMapAreaPassOnly.emplace_back(point);
}

// Read AprilTag vectors
void T1Map2::ReadAprilTagPos(MapPoint point) {
    //Storage for detected AprilTag positions
    point.yaw = point.alt;
    myMapAprilTag.emplace_back(point);
}

// set AprilTag coordinates
int T1Map2::SetAprilTagChrgStation(int stationlength) {
    MapPoint tmppoint;
    tmppoint.lon = 0;
    tmppoint.lat = -1 * stationlength;
    tmppoint.yaw = 0;
    tmppoint.roll = -1000;
    myMapAprilTag.emplace(myMapAprilTag.begin(),tmppoint);
    int length = myMapAprilTag.size();
    for ( int i=0; i<length; i++ )
        ;//printf("T1Map2-SetAprilTagChrgStation loop(%d), Tag=(%lld,%lld,%d)\n", i, myMapAprilTag[i].lon, myMapAprilTag[i].lat, myMapAprilTag[i].yaw);
    return length;
}

// Set Forbiden area of all type
void T1Map2::SetAreaForbidden() {
    int length = myMapForbidden.size();
    printf("T1Map2-SetAreaForbidden begin: %d\n", length);
    for ( int i=0; i<length; i++ ) {
        printf("T1Map2-SetAreaForbidden loop(%d): %d,%d\n", i, (int)myMapForbidden[i].lon, (int)myMapForbidden[i].lat);
        if ( myMapForbidden[i].roll == -1 )
            SetForbiddenRect((int)myMapForbidden[i].lon,(int)myMapForbidden[i].lat,
                myMapForbidden[i].alt,myMapForbidden[i].fixed,myMapForbidden[i].sate);
        else if ( myMapForbidden[i].roll == -2 )
            SetForbiddenCircle((int)myMapForbidden[i].lon,(int)myMapForbidden[i].lat,myMapForbidden[i].alt);
        else
            printf("T1Map2-SetForbiddSetAreaForbiddenenArea error, unknown type:%d\n", myMapForbidden[i].sate);
    }
}

/* Set PassOnly area
 * 2026.05.27: created by Leo
 */
void T1Map2::SetAreaPassOnly() {
    int length = myMapAreaPassOnly.size();
    printf("T1Map2-SetAreaPassOnly begin: %d\n", length);
    for ( int i=0; i<length; i++ ) {
        printf("T1Map2-SetAreaPassOnly loop(%d): %d,%d\n", i, (int)myMapAreaPassOnly[i].lon, (int)myMapAreaPassOnly[i].lat);
        if ( myMapAreaPassOnly[i].roll == -101 )
            SetPassOnlyRect((int)myMapAreaPassOnly[i].lon,(int)myMapAreaPassOnly[i].lat,
                myMapAreaPassOnly[i].alt,myMapAreaPassOnly[i].fixed,myMapAreaPassOnly[i].sate);
        else if ( myMapAreaPassOnly[i].roll == -102 )
            SetPassOnlyCircle((int)myMapAreaPassOnly[i].lon,(int)myMapAreaPassOnly[i].lat,myMapAreaPassOnly[i].alt);
        else
            printf("T1Map2-SetForbiddSetAreaPassOnlyenArea error, unknown type:%d\n", myMapAreaPassOnly[i].sate);
    }
}

/* Set PassOnly area of Rectangle
 * 2026.05.27: created by Leo
 */
void T1Map2::SetPassOnlyRect(int x1, int y1, int x2, int y2, int angle) {
    printf("T1Map2-SetPassOnlyRect begin: (%d,%d), (%d,%d), %d\n", x1,y1, x2,y2, angle);
    int x,y,gx,gy;
    for ( int i=x1; i<=x2; i++ ) {
        for ( int j=y1; j<=y2; j++ ) {
            x = i;
            y = j;
            if ( x < mapMin.lon ) continue;
            if ( y < mapMin.lat ) continue;
            if ( x > mapMax.lon ) continue;
            if ( y > mapMax.lat ) continue;
            gx = ( x - mapMin.lon ) / meWidthGrid + 1;
            gy = ( y - mapMin.lat ) / meWidthGrid + 1;
            //printf("T1Map2-SetPassOnlyRect loop: i,j=(%d,%d), (%d,%d)=VALUEGPASSONLY\n", i, j, gx, gy);
            myGrid[gx][gy] = VALUEGPASSONLY;
        }
    }
}

// Set Forbiden area of Rectangle
void T1Map2::SetForbiddenRect(int x1, int y1, int x2, int y2, int angle) {
    int x,y,gx,gy;
    for ( int i=x1; i<=x2; i++ ) {
        for ( int j=y1; j<=y2; j++ ) {
            x = i;
            y = j;
            if ( x < mapMin.lon ) continue;
            if ( y < mapMin.lat ) continue;
            if ( x > mapMax.lon ) continue;
            if ( y > mapMax.lat ) continue;
            gx = ( x - mapMin.lon ) / meWidthGrid + 1;
            gy = ( y - mapMin.lat ) / meWidthGrid + 1;
            //printf("T1Map2-SetForbiddenRect loop: i,j=(%d,%d), (%d,%d)=VALUEGFORB\n", i, j, gx, gy);
            myGrid[gx][gy] = VALUEGFORB;
        }
    }
}

/* Set PassOnly area of a circle in the grid map
 * 2026.05.27: created by Leo
 * Parameters: cx - longitude of the circle's center (actual coordinate)
 *             cy - latitude of the circle's center (actual coordinate)
 *             r  - radius of the circle (actual coordinate unit)
 */
void T1Map2::SetPassOnlyCircle(int cx, int cy, int r) {  
    // Define traversal boundary: expand radius r around the center to reduce invalid loops
    int xStart = cx - r;
    int xEnd = cx + r;
    int yStart = cy - r;
    int yEnd = cy + r;

    int x, y, gx, gy;
    // Traverse all actual coordinate points covered by the circle
    for (int i = xStart; i <= xEnd; ++i) {
        for (int j = yStart; j <= yEnd; ++j) {
            x = i;
            y = j;

            // Skip actual coordinates outside the map boundary
            if (x < mapMin.lon || x > mapMax.lon) continue;
            if (y < mapMin.lat || y > mapMax.lat) continue;

            // Judge if the current point is inside the circle (circle equation: (x-cx)² + (y-cy)² ≤ r²)
            long long dx = x - cx; // Use long long to avoid int overflow
            long long dy = y - cy;
            long long distanceSq = dx * dx + dy * dy; // Squared distance (avoid square root for efficiency)
            long long rSq = (long long)r * r;
            if (distanceSq > rSq) continue; // Skip points outside the circle

            // Convert actual coordinates to grid coordinates (same formula as rectangle function)
            gx = (x - mapMin.lon) / meWidthGrid + 1;
            gy = (y - mapMin.lat) / meWidthGrid + 1;

            // Grid coordinate validity check (prevent array out of bounds)
            // Add judgment if the grid array has fixed size, e.g.: if (gx > maxGx || gy > maxGy) continue;

            //printf("T1Map2-SetPassOnlyCircle loop: (x,y)=(%d,%d), (gx,gy)=(%d,%d)=VALUEGPASSONLY\n", x, y, gx, gy);

            // Mark the grid as PassOnly area (VALUEGPASSONLY = pass status)
            myGrid[gx][gy] = VALUEGPASSONLY;
        }
    }
}

// Set forbidden area of a circle in the grid map
// Parameters: cx - longitude of the circle's center (actual coordinate)
//             cy - latitude of the circle's center (actual coordinate)
//             r  - radius of the circle (actual coordinate unit)
void T1Map2::SetForbiddenCircle(int cx, int cy, int r) {
    // Define traversal boundary: expand radius r around the center to reduce invalid loops
    int xStart = cx - r;
    int xEnd = cx + r;
    int yStart = cy - r;
    int yEnd = cy + r;

    int x, y, gx, gy;
    // Traverse all actual coordinate points covered by the circle
    for (int i = xStart; i <= xEnd; ++i) {
        for (int j = yStart; j <= yEnd; ++j) {
            x = i;
            y = j;

            // Skip actual coordinates outside the map boundary
            if (x < mapMin.lon || x > mapMax.lon) continue;
            if (y < mapMin.lat || y > mapMax.lat) continue;

            // Judge if the current point is inside the circle (circle equation: (x-cx)² + (y-cy)² ≤ r²)
            long long dx = x - cx; // Use long long to avoid int overflow
            long long dy = y - cy;
            long long distanceSq = dx * dx + dy * dy; // Squared distance (avoid square root for efficiency)
            long long rSq = (long long)r * r;
            if (distanceSq > rSq) continue; // Skip points outside the circle

            // Convert actual coordinates to grid coordinates (same formula as rectangle function)
            gx = (x - mapMin.lon) / meWidthGrid + 1;
            gy = (y - mapMin.lat) / meWidthGrid + 1;

            // Grid coordinate validity check (prevent array out of bounds)
            // Add judgment if the grid array has fixed size, e.g.: if (gx > maxGx || gy > maxGy) continue;

            //printf("T1Map2-SetForbiddenCircle loop: (x,y)=(%d,%d), (gx,gy)=(%d,%d)=VALUEGFORB\n", x, y, gx, gy);

            // Mark the grid as forbidden area (VALUEGFORB = forbidden status)
            myGrid[gx][gy] = VALUEGFORB;
        }
    }
}

/* Set Forbidden points outside the border
 */
void T1Map2::SetGridOutside() {
    int i=0, j=0;

    // Initialize all non-border cells to unknown
    for ( i=0; i<meGridMaxX; i++ ) {
        for ( j=0; j<meGridMaxY; j++ ) {
            if ( myGrid[i][j] != VALUEGBORD ) {
                myGrid[i][j] = -1;  // Mark as unknown
            }
        }
    }

    // Find a seed point on the border to start flood fill
    bool foundSeed = false;
    int seedX = 0, seedY = 0;
    for ( i = 0; i < meGridMaxX && !foundSeed; i++ ) {
        for ( j = 0; j < meGridMaxY && !foundSeed; j++ ) {
            if ( myGrid[i][j] == VALUEGBORD ) {
                // Check if there's space outside border to start flood fill
                if ( i > 0 && myGrid[i-1][j] == -1 ) {
                    seedX = i-1;
                    seedY = j;
                    foundSeed = true;
                } else if ( i < meGridMaxX - 1 && myGrid[i+1][j] == -1 ) {
                    seedX = i+1;
                    seedY = j;
                    foundSeed = true;
                } else if ( j > 0 && myGrid[i][j-1] == -1 ) {
                    seedX = i;
                    seedY = j-1;
                    foundSeed = true;
                } else if ( j < meGridMaxY - 1 && myGrid[i][j+1] == -1 ) {
                    seedX = i;
                    seedY = j+1;
                    foundSeed = true;
                }
            }
        }
    }

    // If no seed found on border, use (0,0) as fallback
    if ( !foundSeed ) {
        seedX = 0;
        seedY = 0;
    }

    // Flood fill: mark all reachable unknown cells as outside (forbidden)
    // Use queue-based BFS to avoid stack overflow with recursion
    std::queue<std::pair<int, int>> q;
    if ( seedX >= 0 && seedX < meGridMaxX && 
         seedY >= 0 && seedY < meGridMaxY && 
         myGrid[seedX][seedY] == -1 ) {
        q.push({seedX, seedY});
        myGrid[seedX][seedY] = VALUEGFORB;
    }

    int dx[] = {-1, 1, 0, 0};
    int dy[] = {0, 0, -1, 1};

    while ( !q.empty() ) {
        auto curr = q.front();
        q.pop();
        int x = curr.first;
        int y = curr.second;

        for ( int k = 0; k < 4; k++ ) {
            int nx = x + dx[k];
            int ny = y + dy[k];
            if ( nx >= 0 && nx < meGridMaxX && 
                 ny >= 0 && ny < meGridMaxY && 
                 myGrid[nx][ny] == -1 ) {
                myGrid[nx][ny] = VALUEGFORB;
                q.push({nx, ny});
            }
        }
    }

    // Now mark all remaining unknown cells as inside (passable)
    for ( i = 0; i < meGridMaxX; i++ ) {
        for ( j = 0; j < meGridMaxY; j++ ) {
            if ( myGrid[i][j] == -1 ) {
                myGrid[i][j] = VALUEGGRASS;
            }
        }
    }
}

/* Set border points in the grid map
 */
void T1Map2::SetGridBorder() {
    int length = myMapBorder.size(), x, y;
    MapPoint point;
    //printf("T1Map2-SetGridBorder begin, length=%d, Min(%lld,%lld), Max(%lld,%lld)\n", length, mapMin.lon,mapMin.lat, mapMax.lon,mapMax.lat);
    for ( int i=0; i<length; i++ ) {
        point = GetPoint('B',i);
        x = (point.lon-mapMin.lon)/meWidthGrid + 1;
        y = (point.lat-mapMin.lat)/meWidthGrid + 1;
        //printf("T1Map2-SetGridBorder loop(%d), i=%d, (%d,%d)=VALUEGBORD\n", length, i, x, y);//, myGrid[x][y]);
        myGrid[x][y] = VALUEGBORD;
    }
}

/* Set the grid inside the border points
 * 2026.04.13: created  by Leo
 */
void T1Map2::SetGridBorderInside() {
    int length = myMapBorder.size(), x, y;
    MapPoint point;
    //printf("T1Map2-SetGridBorderInside begin, length=%d, Min(%lld,%lld), Max(%lld,%lld)\n", length, mapMin.lon,mapMin.lat, mapMax.lon,mapMax.lat);
    for ( int i=0; i<length; i++ ) {
        point = GetPoint('B',i);
        x = (point.lon-mapMin.lon)/meWidthGrid + 1;
        y = (point.lat-mapMin.lat)/meWidthGrid + 1;
        //printf("T1Map2-SetGridBorderInside loop(%d), i=%d, (%d,%d)=VALUEGBORD, meGridMaxXY=(%d,%d)\n", length, i, x, y, meGridMaxX, meGridMaxY);
        for ( int j=x-meWidthBorder; j<=x+meWidthBorder; j++ ) {
            if ( j<0 || j>=meGridMaxX ) continue;
            for ( int k=y-meWidthBorder; k<=y+meWidthBorder; k++ ) {
                if ( k<0 || k>=meGridMaxY ) continue;
                //printf("T1Map2-SetGridBorderInside loop inside i-x-y=(%d,%d,%d), j-k=(%d,%d)\n", i, x,y, j, k);//, myGrid[x][y]);
                if ( myGrid[j][k] != VALUEGFORB )
                    myGrid[j][k] = VALUEGBORD;
            }
        }
    }
}

// creat the grid map
int T1Map2::GetGrid() {
    //printf("T1Map2-GetGrid begin, c=%d,g=%d\n", meWidthCutter, meWidthGrid);
    // calculate the grid map's size
    meGridMaxX = ( mapMax.lon - mapMin.lon ) / meWidthGrid + 3;
    meGridMaxY = ( mapMax.lat - mapMin.lat ) / meWidthGrid + 3;
    if ( meGridMaxX <= 0 || meGridMaxY <= 0) {
        printf("T1Map2-GetGrid malloc error, GridMaxX=%d, GridMaxY=%d\n", meGridMaxX, meGridMaxY);
        T1Utils::LogBug("T1Map2-GetGrid malloc error, GridMax <= 0\n");
        return -1;
    }
    //printf("T1Map2-GetGrid size max(%lld,%lld),min(%lld,%lld), MaxX=%d,MxzY=%d\n", mapMax.lon, mapMax.lat, mapMin.lon, mapMin.lat, meGridMaxX, meGridMaxY);

    // alloc the grid map's memory space
    int *dataGrid = (int*)malloc( meGridMaxX * meGridMaxY * sizeof(int) );
    if ( dataGrid == NULL ) {
        T1Utils::LogBug("T1Map2-GetGrid malloc data failed.\n");
    }
    myGrid = (int**)malloc( meGridMaxX * sizeof(int*) );
    if ( myGrid == NULL ) {
        free(dataGrid);
        T1Utils::LogBug("T1Map2-GetGrid malloc data failed.\n");
        return -1;
    }
    for ( int i=0; i<meGridMaxX; i++ ) {
        myGrid[i] = dataGrid + i * meGridMaxY;
    }

    printf("T1Map2-GetGrid end ok, GridMaxX=%d, GridMaxY=%d\n", meGridMaxX, meGridMaxY);
    
    return 1;
}

// Check if the grid coordinate is accessible (in bounds and not forbidden)
bool T1Map2::isGridAccessible(int x, int y) {
    // Check if lon/lat is within the grid boundary
    //if (x < 0 || x > meGridMaxX || y < 0 || y > meGridMaxY) {
    if ( x < mapMin.lon || x > mapMax.lon || y < mapMin.lat || y > mapMax.lat ) {
        return false;
    }
    // Forbidden area is marked as VALUEGFORB: cast to unsigned char to avoid signed char issue
    int gx = (x-mapMin.lon)/meWidthGrid + 1;
    int gy = (y-mapMin.lat)/meWidthGrid + 1;
    //return ( myGrid[gx][gy] != VALUEGFORB );
    if ( myGrid[gx][gy]==VALUEGFORB ) return false;
    else if ( myGrid[gx][gy]==VALUEGBORD ) return false;
    else if ( myGrid[gx][gy]==VALUEGCUT ) return false;         // added for GetRoute3()
    else if ( myGrid[gx][gy]<=VALUEGORBSTACLE ) return false;   // added for GetRoute3()
    else if ( myGrid[gx][gy]==VALUEGPASSONLY ) return false;   // added for passing bridge
    else return true;
}

/* Generate line segments based on the specified angle (degree) and store in the vector
 * Segments use MapPoint for start/end coordinates
 * 2025.11.30: created by Leo, the order is -N...0...N
 * 2026.05.07: modified by Leo, the order is 0..N,-1...-N
 */
void T1Map2::GetRouteLines(int angleDeg) {
    //printf("T1Map2-GetRouteLines begin. C=%d,G=%d,A=%d\n", meWidthCutter, meWidthGrid, angleDeg);
    myRoutePlan2.clear();  // Clear existing segments before new generation
    myRoutePlanLine.clear();  // Clear existing segments before new generation

    // Convert angle from degrees to radians for trigonometric calculations
    double angleRad = angleDeg * M_PI / 180.0;
    // Unit vector along the line direction (dx = lon, dy = lat)
    double dirDx = sin(angleRad);  // x direction (matches MapPoint.lon)
    double dirDy = cos(angleRad);  // y direction (matches MapPoint.lat)
    // Unit vector perpendicular to the line direction (for generating parallel lines)
    double perpDx = -dirDy;
    double perpDy = dirDx;
    //printf("T1Map2-GetRouteLines step 1. dirDxy=(%.3f,%.3f), perpDxy=(%.3f,%.3f),\n", dirDx,dirDy, perpDx, perpDy);

    // Calculate maximum grid diagonal to cover the entire map
    int maxDiag = sqrt( pow((mapMax.lon-mapMin.lon), 2) + pow((mapMax.lat-mapMin.lat), 2) ) + 1;
    // Number of parallel lines: cover both positive and negative offset directions
    int widthInterval = meWidthCutter-meWidthGrid;
    int maxOffset = ( maxDiag / widthInterval ) + 2;

    //printf("T1Map2-GetRouteLines step 2. maxGridXY=(%d,%d), maxDiag=%d, maxOffset=%d.\n", meGridMaxX, meGridMaxY, maxDiag, maxOffset);
    LineSegment tmpLine;
    // Traverse parallel lines with fixed interval offset, from 0 to N
    for ( int offset = 0; offset <= maxOffset; offset++ ) {
        GetRouteLinesSingle(offset, perpDx,perpDy, dirDx,dirDy, maxDiag, widthInterval, tmpLine);
    }
    for ( int offset = -1; offset >= -maxOffset; offset-- ) {
        GetRouteLinesSingle(offset, perpDx,perpDy, dirDx,dirDy, maxDiag, widthInterval, tmpLine);
    }
    printf("T1Map2-GetRouteLines end, LineLength=%ld, PointLength=%ld, cutter=%d, grid=%d, angle=%d\n", myRoutePlanLine.size(), myRoutePlan2.size(), meWidthCutter, meWidthGrid, angleDeg);
}

/* creat single line, to simplfied the GerRoutuLines()
 * 2026.05.07: created by Leo
 */
void T1Map2::GetRouteLinesSingle(int offset, double perpDx, double perpDy, double dirDx, double dirDy,
                                int maxDiag, int widthInterval, LineSegment& tmpLine) {
    // Calculate base offset of the current parallel line
    double baseX = offset * (meWidthCutter-meWidthGrid) * perpDx;
    double baseY = offset * (meWidthCutter-meWidthGrid) * perpDy;
    bool isInPassableArea = false;  // Flag: whether current point is in passable area
    //printf("T1Map2-GetRouteLines offset(%d)\n", offset);

    // Traverse along the line direction to scan grid points
    for ( int j = -maxDiag; j <= maxDiag; j++ ) {
        // Calculate current grid coordinate (rounded to integer)
        int currX = static_cast<int>(round(baseX + j * dirDx));
        int currY = static_cast<int>(round(baseY + j * dirDy));
        //printf("T1Map2-generateLineSegments step(%d)\n", j);
        // Check if current grid is accessible
        if (isGridAccessible(currX, currY)) {
            if (!isInPassableArea) {
                // Enter passable area: record segment start point
                tmpLine.Start.lon = currX;
                tmpLine.Start.lat = currY;
                isInPassableArea = true;
                //printf("T1Map2-GetRouteLines[%ld]:  (%d,%3d) start(%3d,%4d), ", myRoutePlanLine.size(), offset, j, currX, currY);
            }
            // Update segment end point (extend the segment)
            tmpLine.End.lon = currX;
            tmpLine.End.lat = currY;
        } else {
            if (isInPassableArea) {
                // Exit passable area: save the segment to the vector
                myRoutePlanLine.emplace_back(tmpLine);
                isInPassableArea = false;
                //printf("(%d,%3d) end(%3d,%4d)\n", offset, j, currX, currY);
            }
        }
        //printf("T1Map2-GetRouteLines loop end step(%d,%d)\n", offset, j);
    }
    // Save the last segment if it ends inside the passable area
    if (isInPassableArea) {
        myRoutePlanLine.emplace_back(tmpLine);
    }
}
/*void T1Map2::GetRouteLinesOld(int angleDeg) {
    //printf("T1Map2-GetRouteLines begin. C=%d,G=%d,A=%d\n", meWidthCutter, meWidthGrid, angleDeg);
    myRoutePlan2.clear();  // Clear existing segments before new generation
    myRoutePlanLine.clear();  // Clear existing segments before new generation

    // Convert angle from degrees to radians for trigonometric calculations
    double angleRad = angleDeg * M_PI / 180.0;
    // Unit vector along the line direction (dx = lon, dy = lat)
    double dirDx = sin(angleRad);  // x direction (matches MapPoint.lon)
    double dirDy = cos(angleRad);  // y direction (matches MapPoint.lat)
    // Unit vector perpendicular to the line direction (for generating parallel lines)
    double perpDx = -dirDy;
    double perpDy = dirDx;
    //printf("T1Map2-GetRouteLines step 1. dirDxy=(%.3f,%.3f), perpDxy=(%.3f,%.3f),\n", dirDx,dirDy, perpDx, perpDy);

    // Calculate maximum grid diagonal to cover the entire map
    int maxDiag = sqrt( pow((mapMax.lon-mapMin.lon), 2) + pow((mapMax.lat-mapMin.lat), 2) ) + 1;
    // Number of parallel lines: cover both positive and negative offset directions
    int maxOffset = ( maxDiag / (meWidthCutter-meWidthGrid) ) + 2;

    //printf("T1Map2-GetRouteLines step 2. maxGridXY=(%d,%d), maxDiag=%d, maxOffset=%d.\n", meGridMaxX, meGridMaxY, maxDiag, maxOffset);
    LineSegment tmpLine;
    // Traverse all parallel lines with fixed interval offset
    for ( int offset = -maxOffset; offset <= maxOffset; offset++ ) {
        // Calculate base offset of the current parallel line
        double baseX = offset * (meWidthCutter-meWidthGrid) * perpDx;
        double baseY = offset * (meWidthCutter-meWidthGrid) * perpDy;
        bool isInPassableArea = false;  // Flag: whether current point is in passable area
        //printf("T1Map2-GetRouteLines offset(%d)\n", offset);

        // Traverse along the line direction to scan grid points
        for ( int j = -maxDiag; j <= maxDiag; j++ ) {
            // Calculate current grid coordinate (rounded to integer)
            int currX = static_cast<int>(round(baseX + j * dirDx));
            int currY = static_cast<int>(round(baseY + j * dirDy));
            //printf("T1Map2-generateLineSegments step(%d)\n", j);
            // Check if current grid is accessible
            if (isGridAccessible(currX, currY)) {
                if (!isInPassableArea) {
                    // Enter passable area: record segment start point
                    tmpLine.Start.lon = currX;
                    tmpLine.Start.lat = currY;
                    isInPassableArea = true;
                    //printf("T1Map2-GetRouteLines[%ld]:  (%d,%3d) start(%3d,%4d), ", myRoutePlanLine.size(), offset, j, currX, currY);
                }
                // Update segment end point (extend the segment)
                tmpLine.End.lon = currX;
                tmpLine.End.lat = currY;
            } else {
                if (isInPassableArea) {
                    // Exit passable area: save the segment to the vector
                    myRoutePlanLine.emplace_back(tmpLine);
                    isInPassableArea = false;
                    //printf("(%d,%3d) end(%3d,%4d)\n", offset, j, currX, currY);
                }
            }
            //printf("T1Map2-GetRouteLines loop end step(%d,%d)\n", offset, j);
        }
        // Save the last segment if it ends inside the passable area
        if (isInPassableArea) {
            myRoutePlanLine.emplace_back(tmpLine);
        }
    }
    printf("T1Map2-GetRouteLines end, LineLength=%ld, PointLength=%ld, cutter=%d, grid=%d, angle=%d\n", myRoutePlanLine.size(), myRoutePlan2.size(), meWidthCutter, meWidthGrid, angleDeg);
}*/

/*/ Print all line segments in the vector (line number + MapPoint start/end coordinates)
void T1Map2::printRouteLines() {
    // Print total number of segments
    printf("T1Map2-printRouteLines, Total generated line segments: %zu\n", myRoutePlan2.size());
    // Traverse the vector and print each segment with line number
    for (size_t i = 0; i < myRoutePlan2.size(); ++i) {
        //const LineSegment& seg = segments[i];
        printf("T1Map2-printRouteLines loop(%zu) | Start: (lon=%lld, lat=%lld) | End: (lon=%lld, lat=%lld)\n",
               i + 1,  // Line number starts from 1
               myRoutePlanLine[i].Start.lon, myRoutePlanLine[i].Start.lat,
               myRoutePlanLine[i].End.lon, myRoutePlanLine[i].End.lat);
    }
}*/

/* this will be run when obstacle found, new route lines will skip obstacle and cutted
 * 2026.05.20: created by Leo
 */ 
int T1Map2::GetRoute3(int angle) {
    printf("T1Map2-GetRoute3 begin. C=%d,G=%d,A=%d\n", meWidthCutter, meWidthGrid, angle);
    if ( myMapRemote[0].lon == -1 && myMapRemote[0].lat == -1 ) {
        if ( myMapRemote[0].alt != -1 ) meWidthCutter=myMapRemote[0].alt;
        if ( myMapRemote[0].fixed != -1 ) meWidthGrid=myMapRemote[0].fixed;
        if ( myMapRemote[0].sate != -1 ) {
            meAnglePlan=myMapRemote[0].sate;
            angle = meAnglePlan;
        }
    }
    printf("T1Map2-GetRoute3 check if demo map. C=%d,G=%d,A=%d\n", meWidthCutter, meWidthGrid, angle);

    GetRouteLines(angle);
    SetGridLineRoute();

    return 1;
}

int T1Map2::GetRoute2(int angle) {
    printf("T1Map2-GetRoute2 begin. C=%d,G=%d,A=%d\n", meWidthCutter, meWidthGrid, angle);
    if ( myMapRemote[0].lon == -1 && myMapRemote[0].lat == -1 ) {
        if ( myMapRemote[0].alt != -1 ) meWidthCutter=myMapRemote[0].alt;
        if ( myMapRemote[0].fixed != -1 ) meWidthGrid=myMapRemote[0].fixed;
        if ( myMapRemote[0].sate != -1 ) {
            meAnglePlan=myMapRemote[0].sate;
            angle = meAnglePlan;
        }
    }
    printf("T1Map2-GetRoute2 check if demo map. C=%d,G=%d,A=%d\n", meWidthCutter, meWidthGrid, angle);

    int ret = GetGrid();
    if ( ret <= 0 ) return ret;

    SetGridBorder();
    SetGridOutside();
    SetGridBorderInside();
    SetAreaForbidden();
    SetAreaPassOnly();
    //SetAprilTagArea();

    GetRouteLines(angle);
    SetGridLineRoute();

    //findA();

    printf("T1Map2-GetRoute2 end, LineSize=%ld\n", myRoutePlanLine.size());
    return myRoutePlanLine.size();
}

int T1Map2::GetRoute(int type) {
    //printf("T1Map2-GetRoute begin.\n");

    myMapForbidden.clear(); // this vector will be append in GetRemote()

    int ret = GetBorder();
    if ( ret <= 0 ) return ret;

    //if ( type > 10 ) {
    if ( type >= 0 ) {
        ret = GetRoute2(meAnglePlan);
        //return ret;
    }

    GetScaleLonLat();
    if ( type==0 )
        type=meTypeRoutePlan;
//printf("T1Map2-GetRoute begin, type=%d, ret=%d, Min(%lld,%lld), Max(%lld,%lld)\n", type, ret, mapMin.lon,mapMin.lat, mapMax.lon,mapMax.lat);
    // scan latitude
    MapPoint tmp; // all is 0,0,0,0,0,0
    myRoutePlan.clear();
    myRoutePlan.emplace_back(tmp); // myBase should be 0,0,0,0,0,0 in the relative coordinates.
//printf("T1Map2-GetRoute step0: size:%ld, Base:(%lld, %lld)\n", myRoutePlan.size(), myBase.lon, myBase.lat);// meWorkline << mapMin.lon << mapMax.lon;
    switch (type) {
    case 1:
        RouteZLon();
        break;
    case 2:
        RouteZLat();
        break;
    case 3:
        RouteZLon();
        RouteZLat();
        break;
    case 5:
        RouteALon();
        break;
    case 6:
        RouteALat();
        break;
    case 7:
        RouteALon();
        RouteALat();
        break;
    default:
        std::cout << "T1Map2-GetRoute step4, error type:" << type;
        break;
    }

    myRoutePlan.emplace_back(tmp); // add the myBase=0 again
//printf("T1Map2-GetRoute step5, size=%ld, Base:(%lld, %lld)\n", myRoutePlan.size(), myBase.lon, myBase.lat);// meWorkline << mapMin.lon << mapMax.lon;

    for ( int i=0; i<myRoutePlan.size(); i++ ) {
        ;//printf("T1Map2-GetRoute step6, all RoutePlan: %ld-%d, (%lld, %lld, %d)\n", myRoutePlan.size(), i, myRoutePlan[i].lon, myRoutePlan[i].lat, myRoutePlan[i].alt);
    }

    // scan longitude

    printf("T1Map2-GetRoute end, size=%ld, Min(%lld,%lld), Max(%lld,%lld)\n", myRoutePlan.size(), mapMin.lon, mapMin.lat, mapMax.lon, mapMax.lat);

    return myRoutePlan.size();
}

MapPoint T1Map2::GetPoint2(char name, int val) {
    switch (name) {
    case 'A':
        return myMapAprilTag[val];
        break;
    case '2':
        return myRoutePlan2[val];
        break;
    case 'L':
        return myRoutePlanLine[val].Start;
        break;
    case 'l':
        return myRoutePlanLine[val].End;
        break;
    default:
        printf("T1Map2-GetPoint2 error type[%c,%d]\n", name, val);
        return myBase;
        break;
    }
}

// void T1Map2::SetValue(char name, int val) {
//     switch (name) {
//     case 'c':
//         meWidthCutter = val;
//         break;
//     case 'g':
//         meWidthGrid = val;
//         break;
//     case 'a':
//         meAnglePlan = val;
//         break;
//     default:
//         printf("T1Map2-SetValue error, unknown name:[%c,%d]\n", name, val);
//         break;
//     }
//     printf("T1Map2-SetValue end, name:[%c,%d], C-G-A=(%d,%d,%d)\n", name, val, meWidthCutter, meWidthGrid, meAnglePlan);
// }

void T1Map2::SetValue(char name, int val) {
    bool needRegenerate = false;
    switch (name) {
    case 'c':
        meWidthCutter = val;  // Set from factory parameter StepLinec
        break;
    case 'g':
        meWidthGrid = val;  // Set from factory parameter StepLineg
        break;
    case 'a':
        meAnglePlan = val;  // Set from factory parameter StepLinea
        break;
    case 'b':
        meWidthBorder = val;  // Set from factory parameter BorderWidth
        break;
    default:
        printf("T1Map2-SetValue error, unknown name:[%c,%d]\n", name, val);
        break;
    }
    //printf("T1Map2-SetValue end, name:[%c,%d], C-G-A=(%d,%d,%d)\n", name, val, meWidthCutter, meWidthGrid, meAnglePlan);
    
    // When meWidthGrid, meWidthCutter, or meAnglePlan changes, the grid and routes must be regenerated
    /*/ because: meWidthGrid changes grid size, meWidthCutter changes line spacing, meAnglePlan changes route direction
    if (needRegenerate && mapMin.lon != 0 && mapMax.lon != 0) {
        // Only regenerate if map is already loaded (mapMin/max are valid)
        printf("T1Map2-SetValue: Regenerating grid and routes due to parameter change (from factory)\n");
        GetRoute(0);  // Regenerate routes with new parameters
    }*/
}


// ************** A star follows *********************

// Heuristic function: Manhattan distance (admissible for grid paths)
int T1Map2::AstarManhattan(const GridNode* node, const GridNode& end) {
    return abs(node->x - end.x) + abs(node->y - end.y);
}

// Check if a coordinate (x,y) is valid: within bounds and passable
bool T1Map2::AstarIsValid(int x, int y) {
    // Validate grid boundaries and passability (myGrid[x][y] == 0 means passable)
    //printf("T1Map2-is_valid: point(%d,%d)=%d\n", x, y, myGrid[x][y]);
    //return (x >= 0 && x <= meGridMaxX && y >= 0 && y <= meGridMaxY && myGrid[x][y] == 0);
    if ( x<0 || x>=meGridMaxX || y<0 || y>=meGridMaxY )
        return false;
    int col = myGrid[x][y];
    //if ( col == VALUEGFORB || col <= VALUEGORBSTACLE )//|| col == VALUEGBORD )
    if ( col == VALUEGFORB || col <= VALUEGORBSTACLE || col == VALUEGBORD )
        return false;
    return true;
}

// A* algorithm implementation
// Input: start (start_x, start_y) and end (end_x, end_y) coordinates
// Output: Path as a vector of (x,y) pairs (empty if no path exists)
void ClearOpenList(std::priority_queue<GridNode*, std::vector<GridNode*>, CompareNode>& open_list)
{
    while (!open_list.empty())
    {
        GridNode* temp = open_list.top();
        open_list.pop();
        delete temp;
    }
}
/* return: result.g_value>0, path found;  =0, start and end are same;  <0, no path found;
 * 2026.05.20: created by Leo(+AI)
 */
AStarResult T1Map2::AstarSearch(int start_x, int start_y, int end_x, int end_y) {
    AStarResult result;

    if (!AstarIsValid(end_x, end_y)) {
        printf("T1Map2-AstarSearch Invalid end point (obstacle or out of grid bounds) end(%d,%d)\n", end_x, end_y);
        return result; //g_value is -1 by default, indicating no path found
    }

    int actual_start_x = start_x;
    int actual_start_y = start_y;
    bool is_original_start_valid = AstarIsValid(start_x, start_y);

    if (!is_original_start_valid) {
        printf("Warning T1Map2-AstarSearch: Start point is in forbidden area, searching for the nearest valid point using BFS...\n");
        
        const std::vector<std::pair<int, int>> directions = {
            {-1, 0}, {1, 0}, {0, -1}, {0, 1},
            {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
        };
        std::queue<std::pair<int, int>> bfs_queue;
        std::unordered_map<int, std::unordered_map<int, bool>> visited;
        
        bfs_queue.push({start_x, start_y});
        visited[start_x][start_y] = true;
        bool find_alternative = false;
        
        while (!bfs_queue.empty() && !find_alternative) {
            auto current = bfs_queue.front();
            bfs_queue.pop();
            
            for (const auto& dir : directions) {
                int new_x = current.first + dir.first;
                int new_y = current.second + dir.second;
                
                if (new_x >= 0 && new_y >= 0 && !visited[new_x][new_y]) {
                    visited[new_x][new_y] = true;
                    
                    if (AstarIsValid(new_x, new_y)) {
                        actual_start_x = new_x;
                        actual_start_y = new_y;
                        find_alternative = true;
                        break;
                    }
                    
                    bfs_queue.push({new_x, new_y});
                }
            }
        }

        if (!find_alternative) {
            printf("T1Map2-AstarSearch: No valid area reachable from the start point\n");
            return result; //g_value is -1 by default, indicating no path found
        } else {
            printf("T1Map2-AstarSearch: Alternative start point found (%d,%d) (Original start point: (%d,%d))\n",
                   actual_start_x, actual_start_y, start_x, start_y);
        }
    }

    if (actual_start_x == end_x && actual_start_y == end_y) {
        printf("T1Map2-AstarSearch: Start point(%d,%d) coincides with end point(%d,%d), no need to search\n",
            actual_start_x, actual_start_y, end_x, end_y);
        result.has_path = true;
        result.path.emplace_back(actual_start_x, actual_start_y);
        result.g_value = 0;
        return result; //g_value=0, indicating same point of start and end
    }

    std::priority_queue<GridNode*, std::vector<GridNode*>, CompareNode> open_list;
    std::unordered_map<int, std::unordered_map<int, int>> close_list;

    GridNode start_node(actual_start_x, actual_start_y);
    GridNode end_node(end_x, end_y);
    start_node.h = AstarManhattan(&start_node, end_node);
    open_list.push(new GridNode(actual_start_x, actual_start_y, 0, start_node.h));
    close_list[actual_start_x][actual_start_y] = 0;

    const std::vector<std::pair<int, int>> directions = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1},
        {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
    };

    while (!open_list.empty()) {
        GridNode* current = open_list.top();
        open_list.pop();

        if (current->x == end_x && current->y == end_y) {
            result.g_value = current->g;
            result.has_path = true;
            while (current != nullptr) {
                result.path.emplace_back(current->x, current->y);
                GridNode* temp = current;
                current = current->parent;
                delete temp;
            }
            reverse(result.path.begin(), result.path.end());
            ClearOpenList(open_list);
            return result;
        }

        for (const auto& dir : directions) {
            int new_x = current->x + dir.first;
            int new_y = current->y + dir.second;

            if (AstarIsValid(new_x, new_y)) {
                int new_g = current->g + 1;
                if (!close_list.count(new_x) || !close_list[new_x].count(new_y) || new_g < close_list[new_x][new_y]) {
                    close_list[new_x][new_y] = new_g;
                    GridNode temp_node(new_x, new_y);
                    int new_h = AstarManhattan(&temp_node, end_node);
                    open_list.push(new GridNode(new_x, new_y, new_g, new_h, current));
                }
            }
        }
    }

    printf("No path found between start and end.\n");
    ClearOpenList(open_list);
    result.g_value = -1;
    return result;
}

// A* algorithm implementation
// Input: start (start_x, start_y) and end (end_x, end_y) coordinates
// Output: Path as a vector of (x,y) pairs (empty if no path exists)
/*
AStarResult T1Map2::AstarSearch(int start_x, int start_y, int end_x, int end_y) {
    AStarResult result;  // Initialize the result with no path by default

    if (!AstarIsValid(end_x, end_y)) {
        printf("T1Map2-AstarSearch Invalid end point (obstacle or out of grid bounds) end(%d,%d)\n", end_x, end_y);
        return result;
    }

    int actual_start_x = start_x;
    int actual_start_y = start_y;
    bool is_original_start_valid = AstarIsValid(start_x, start_y);

    if (!is_original_start_valid) {
        printf("Warning T1Map2-AstarSearch: Start point is in forbidden area, searching for the nearest valid point using BFS...\n");
        
        const std::vector<std::pair<int, int>> directions = {
            {-1, 0}, {1, 0}, {0, -1}, {0, 1},
            {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
        };
        std::queue<std::pair<int, int>> bfs_queue;
        std::unordered_map<int, std::unordered_map<int, bool>> visited;
        
        bfs_queue.push({start_x, start_y});
        visited[start_x][start_y] = true;
        bool find_alternative = false;
        
        while (!bfs_queue.empty() && !find_alternative) {
            auto current = bfs_queue.front();
            bfs_queue.pop();
            
            for (const auto& dir : directions) {
                int new_x = current.first + dir.first;
                int new_y = current.second + dir.second;
                
                if (new_x >= 0 && new_y >= 0 && !visited[new_x][new_y]) {
                    visited[new_x][new_y] = true;
                    
                    if (AstarIsValid(new_x, new_y)) {
                        actual_start_x = new_x;
                        actual_start_y = new_y;
                        find_alternative = true;
                        break;
                    }
                    
                    bfs_queue.push({new_x, new_y});
                }
            }
        }

        if (!find_alternative) {
            printf("T1Map2-AstarSearch: No valid area reachable from the start point\n");
            return result;
        } else {
            printf("T1Map2-AstarSearch: Alternative start point found (%d,%d) (Original start point: (%d,%d))\n",
                   actual_start_x, actual_start_y, start_x, start_y);
        }
    }

    if (actual_start_x == end_x && actual_start_y == end_y) {
        printf("T1Map2-AstarSearch: Start point(%d,%d) coincides with end point(%d,%d), no need to search\n",
            actual_start_x, actual_start_y, end_x, end_y);
        result.has_path = true;
        result.path.emplace_back(actual_start_x, actual_start_y);
        result.g_value = 0;
        return result;
    }

    // Nodes to be evaluated (sorted by f value)
    std::priority_queue<GridNode*, std::vector<GridNode*>, CompareNode> open_list;  
    // Evaluated nodes (stores minimum g value for each coordinate)
    std::unordered_map<int, std::unordered_map<int, int>> close_list;  

    // Initialize the alternative start node
    GridNode start_node(actual_start_x, actual_start_y);
    start_node.h = AstarManhattan(&start_node, GridNode(end_x, end_y));
    open_list.push(new GridNode(actual_start_x, actual_start_y, 0, start_node.h));
    close_list[actual_start_x][actual_start_y] = 0;

    // 8 possible movement directions (orthogonal + diagonal)
    const std::vector<std::pair<int, int>> directions = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1},  // Orthogonal directions (up, down, left, right) 
        {-1, -1}, {-1, 1}, {1, -1}, {1, 1}  // Diagonal directions
    };

    while (!open_list.empty()) {
        // Extract the node with the lowest f value from the priority queue
        GridNode* current = open_list.top();
        open_list.pop();

        // Check if the current node is the end node
        if (current->x == end_x && current->y == end_y) {
            result.g_value = current->g;  // Store the g value (step count) of the end node directly
            result.has_path = true;       // Mark that a valid path exists
            // Reconstruct the path by backtracking from the end node to the start node
            while (current != nullptr) {
                result.path.emplace_back(current->x, current->y);
                GridNode* temp = current;
                current = current->parent;
                delete temp;  // Free the allocated memory to avoid memory leaks
            }
            reverse(result.path.begin(), result.path.end());  // Reverse to get the start-to-end path order
            return result;
        }

        // Explore all neighboring nodes of the current node
        for (const auto& dir : directions) {
            int new_x = current->x + dir.first;
            int new_y = current->y + dir.second;

            // Check if the neighbor node is valid (passable and within grid bounds)
            if (AstarIsValid(new_x, new_y)) {
                // Calculate the new g value (movement cost = 1; adjust for diagonal movement if needed)
                int new_g = current->g + 1;

                // Update the neighbor node if:
                // 1. It's not in the close list, or
                // 2. A cheaper path (lower g value) to this node is found
                if (!close_list.count(new_x) || !close_list[new_x].count(new_y) || new_g < close_list[new_x][new_y]) {
                    close_list[new_x][new_y] = new_g;
                    int new_h = AstarManhattan(new GridNode(new_x, new_y), GridNode(end_x, end_y));
                    open_list.push(new GridNode(new_x, new_y, new_g, new_h, current));
                }
            }
        }
    }

    // If the open list is empty and the end node is not reached: no path exists
    printf("No path found between start and end.\n");
    
    // Clean up remaining nodes in the open list to avoid memory leaks
    while (!open_list.empty()) {
        GridNode* temp = open_list.top();
        open_list.pop();
        delete temp;
    }

    result.g_value = -1;
    return result;
}*/

// Simplify path by merging collinear points into line segment endpoints
std::vector<std::pair<int, int>> T1Map2::AstarSimplifyPath(const std::vector<std::pair<int, int>>& original_path) {
    if (original_path.size() <= 2) return original_path;  // No simplification needed

    std::vector<std::pair<int, int>> simplified;
    simplified.push_back(original_path[0]);  // Keep start point
    //simplified.push_back(original_path[1]);  // Keep start point

    // Iterate through path, keeping only non-collinear points
    for (size_t i = 1; i < original_path.size() - 1; ++i) {
    //for (size_t i = 2; i < original_path.size() - 1; ++i) {
        const auto& prev = simplified.back();       // Last kept point
        const auto& curr = original_path[i];        // Current point
        const auto& next = original_path[i + 1];    // Next point

        // Keep current point if it's not collinear with prev and next
        // Collinearity condition: (y2 - y1)*(x3 - x2) == (y3 - y2)*(x2 - x1)
        if ( (curr.second-prev.second) * (next.first-curr.first) != (next.second-curr.second) * (curr.first-prev.first) ) {
            simplified.push_back(curr);
        }
    }

    simplified.push_back(original_path.back());  // Keep end point
    return simplified;
}

// Example usage (for testing; integrate with your grid initialization)
int T1Map2::findA() {

    // Find path from (0,0) to (4,4)
    AStarResult astar_result = AstarSearch(6, 11, 8, 27);

    // Simplify path
    std::vector<std::pair<int, int>> simplified = AstarSimplifyPath(astar_result.path);

    // Print results
    if (astar_result.path.empty()) {
        printf("T1Map2-findA No path found.\n");
    } else {
        //printf("T1Map2-findA Original path (%d): ", astar_result.g_value);
        for (const auto& p : astar_result.path) {
            printf("(%d,%d) ", p.first, p.second);
            myGrid[p.first][p.second] = VALUEGASTA;
        }
        
        printf("\nT1Map2-findA Simplified path: ");
        for (const auto& p : simplified)
            printf("(%d,%d) ", p.first, p.second);
        std::cout << std::endl;
    }

    return 0;
}

/* find the Astar path to the nearest point of the next line, push into vectors RoutePlan2
 * return: 0,error        >0,not reverse and the length       <0, reversed and the length
 * 2026.03.09: modified return valus by Leo
 * 2026.05.01: modified by Leo, add debug=0 to controll printf()
 */

int T1Map2::findAstarNear(int reversed, int currX, int currY, int lineNo) {
    int debug=1;
    int flagReverse, retlen;
    int StartGx,StartGy,EndGx,EndGy;
    if (debug>0) printf("T1Map2-findAstarNear begin, flag=%d, (%d,%d), line=%d\n", reversed, currX, currY, lineNo);
    StartGx = (currX-mapMin.lon) / meWidthGrid + 1;
    StartGy = (currY-mapMin.lat) / meWidthGrid + 1;
    if (debug>1) printf("T1Map2-findAstarNear step1, flag=%d, line=%d, Wg=%d, currXY(%d,%d), mapMint(%lld,%lld), StartGxy(%d,%d)\n", reversed, lineNo, meWidthGrid, currX, currY, mapMin.lon, mapMin.lat, StartGx, StartGy);
    
    EndGx = (myRoutePlanLine[lineNo].Start.lon-mapMin.lon) / meWidthGrid + 1;
    EndGy = (myRoutePlanLine[lineNo].Start.lat-mapMin.lat) / meWidthGrid + 1;
    if (debug>1) printf("T1Map2-findAstarNear step2, A*S:(%d,%d)->(%d,%d)\n",StartGx,StartGy,EndGx,EndGy);
    AStarResult resultS = AstarSearch(StartGx,StartGy,EndGx,EndGy);
    EndGx = (myRoutePlanLine[lineNo].End.lon-mapMin.lon) / meWidthGrid + 1;
    EndGy = (myRoutePlanLine[lineNo].End.lat-mapMin.lat) / meWidthGrid + 1;
    if (debug>1) printf("T1Map2-findAstarNear step3, A*E:(%d,%d)->(%d,%d)\n",StartGx,StartGy,EndGx,EndGy);
    AStarResult resultE = AstarSearch(StartGx,StartGy,EndGx,EndGy);
    
    std::vector<std::pair<int, int>> simplified;
    if ( resultS.g_value==-1 && resultE.g_value==-1 ) { // not found the Astar path
        T1Utils::LogBug("T1Map2-findAstarNear failed, no Astar path found.\n");
        return 0;
    } else if ( resultS.g_value > resultE.g_value ) { // which point is near
        //simplified = resultE.path;
        simplified = AstarSimplifyPath(resultE.path);
        for (const auto& p : resultE.path) {
            ;//printf("(%d,%d) ", p.first, p.second);
            //myGrid[p.first][p.second] = VALUEGASTA;
        }
        flagReverse = 1;
        retlen = resultE.g_value * -1;
    } else {
        //simplified = resultS.path;
        simplified = AstarSimplifyPath(resultS.path);
        for (const auto& p : resultS.path) {
            ;//printf("(%d,%d) ", p.first, p.second);
            //myGrid[p.first][p.second] = VALUEGASTA;
        }
        flagReverse = 0;
        retlen = resultS.g_value;
        if (retlen==0) {
            printf("T1Map2-findAstarNear step4, Astar path found but length=0, set retlen=1 to avoid no move.\n");
            retlen=1;
        }
    }

    myRoutePlan2.clear();
    int length = simplified.size();
    if (debug>1) printf("T1Map2-findAstarNear step5, Astar path's length=%d, %ld,%ld\n", length, resultS.path.size(), resultE.path.size());
    if (debug>2) printf("T1Map2-findAstarNear: ");
    for ( int j=0; j<length; j++ ) { // first and last point does not need
    //for (const auto& p : simplified) {
        //printf("(%d: %d,%d) ", j, p.first, p.second);
        MapPoint tmpPoint;
        if ( j==0 ) {
            tmpPoint.lon = currX;
            tmpPoint.lat = currY;
        } else {
            tmpPoint.lon = mapMin.lon + meWidthGrid * (simplified[j].first-1);
            tmpPoint.lat = mapMin.lat + meWidthGrid * (simplified[j].second-1);
        }
        myRoutePlan2.emplace_back(tmpPoint);
        if (debug>2) printf("(%d-%d: %d,%d %lld,%lld) ", length, j, simplified[j].first, simplified[j].second,tmpPoint.lon,tmpPoint.lat);
    }
    if (debug>2) printf(". retlen=%d\n", retlen);
    return retlen;
}

/* Calculate A* path from current position to target point, push into RoutePlan2
 * return: 0,success        -1,error
 * 2026.05.01: modified by Leo, add debug=0 to controll printf()
 */
int T1Map2::findAstarToPoint(int currX, int currY, int targetX, int targetY) {
    int debug=1;
    if (debug>0) printf("T1Map2-findAstarToPoint begin: (%d,%d)->(%d,%d)\n", currX, currY, targetX, targetY);
    int StartGx = (currX - mapMin.lon) / meWidthGrid + 1;
    int StartGy = (currY - mapMin.lat) / meWidthGrid + 1;
    int EndGx = (targetX - mapMin.lon) / meWidthGrid + 1;
    int EndGy = (targetY - mapMin.lat) / meWidthGrid + 1;
    findBorderPointInside(&EndGx,&EndGy);
    
    if (debug>1) printf("T1Map2-findAstarToPoint: grid(%d,%d)->(%d,%d)\n", StartGx, StartGy, EndGx, EndGy);
    AStarResult result = AstarSearch(StartGx, StartGy, EndGx, EndGy);
    
    if (result.g_value == -1 || !result.has_path || result.path.empty()) {
        T1Utils::LogBug("T1Map2-findAstarToPoint failed, no Astar path found.\n");
        return -1;
    }
    
    std::vector<std::pair<int, int>> simplified = AstarSimplifyPath(result.path);
    
    myRoutePlan2.clear();
    int length = simplified.size();
    if (debug>1) printf("T1Map2-findAstarToPoint Astar path's length=%d, simplified=%d\n", result.g_value, length);
    for (int j = 0; j < length; j++) {
        MapPoint tmpPoint;
        tmpPoint.lon = mapMin.lon + meWidthGrid * (simplified[j].first - 1);
        tmpPoint.lat = mapMin.lat + meWidthGrid * (simplified[j].second - 1);
        tmpPoint.yaw = 0;
        myRoutePlan2.emplace_back(tmpPoint);
        if (debug>2) printf("T1Map2-findAstarToPoint path[%d]: grid(%d,%d) -> world(%lld,%lld)\n", j, simplified[j].first, simplified[j].second, tmpPoint.lon, tmpPoint.lat);
    }
    if (debug>0) printf("T1Map2-findAstarToPoint end, Astar length=%d\n", length);
    return 0;
}

/* find a points path along the border
 * return: <=0,error        >0,the length 
 * 2026.04.14: creat by Leo
 * 2026.06.04: rename by Leo
 */
int T1Map2::GetRouteBorder() {
    //return findAstarToPoint(200,200,0,0); // this will report Segmentation fault
    int x1, y1, x2, y2, x3, y3;
    MapPoint pointBorder, pointPlan;

    int length = myMapBorder.size();
    printf("\n=====\nT1Map2-findBorderPoints begin, length=%d, Min(%lld,%lld), Max(%lld,%lld)\n", length, mapMin.lon,mapMin.lat, mapMax.lon,mapMax.lat);
    if ( length==0 ) return -1;
    std::vector<MapPoint> myMapBorderGrid1,myMapBorderGrid2;  // Grid points alone the border

    //findBorderPointClear();

    pointBorder = GetPoint('B',0);
    x1 = (pointBorder.lon-mapMin.lon)/meWidthGrid + 1;
    y1 = (pointBorder.lat-mapMin.lat)/meWidthGrid + 1;
    findBorderPointInside(&x1,&y1);
    printf("T1Map2-findBorderPoints step1, LonLat=(%lld,%lld), XY=(%d,%d)\n", pointBorder.lon, pointBorder.lat, x1, y1);
    pointPlan.lon=x1; pointPlan.lat=y1; pointPlan.yaw=0;
    myMapBorderGrid1.emplace_back(pointPlan);
    
    for ( int i=1; i<length; i++ ) { // find the nearest inside point
        pointBorder = GetPoint('B',i);
        x2 = (pointBorder.lon-mapMin.lon)/meWidthGrid + 1;
        y2 = (pointBorder.lat-mapMin.lat)/meWidthGrid + 1;
        findBorderPointInside(&x2,&y2);
        //printf("T1Map2-findBorderPoints loop1a(%d/%d), LonLat=(%lld,%lld), XY=(%d,%d)\n", length, i, pointBorder.lon, pointBorder.lat, x2, y2);
        if ( x2==x1 && y2==y1 ) continue;
        //printf("T1Map2-findBorderPoints loop1b(%d/%d), LonLat=(%lld,%lld), XY=(%d,%d)\n", length, i, pointBorder.lon, pointBorder.lat, x2, y2);
        x1=x2; y1=y2;
        pointPlan.lon=x1; pointPlan.lat=y1; pointPlan.yaw=0;
        myMapBorderGrid1.emplace_back(pointPlan);
    }
    int length1 = myMapBorderGrid1.size();
    printf("T1Map2-findBorderPoints step2, length(%d->%d), Min(%lld,%lld), Max(%lld,%lld)\n", length,length1, mapMin.lon,mapMin.lat, mapMax.lon,mapMax.lat);
    if ( length1==0 ) return -1;

    myMapBorderGrid2.emplace_back(myMapBorderGrid1[0]);
    for ( int i=1; i<length1; i++ ) { // remove the duplicate points
        int j=0;
        //printf("T1Map2-findBorderPoints loop2a(%d,%d), LonLat=(%lld,%lld)\n", i,j, myMapBorderGrid1[i].lon,myMapBorderGrid1[i].lat);
        if ( myGrid[myMapBorderGrid1[i].lon][myMapBorderGrid1[i].lat] == VALUEGFORB ) continue;
        //printf("T1Map2-findBorderPoints loop2b(%d,%d), LonLat=(%lld,%lld)\n", i,j, myMapBorderGrid1[i].lon,myMapBorderGrid1[i].lat);
        int length2 = myMapBorderGrid2.size();
        for ( j=0; j<length2; j++ ) {
            if ( myMapBorderGrid2[j].lon==myMapBorderGrid1[i].lon && myMapBorderGrid2[j].lat==myMapBorderGrid1[i].lat )
                break;
        }
        if ( j!=length2 ) continue;
        myMapBorderGrid2.emplace_back(myMapBorderGrid1[i]);
        //printf("T1Map2-findBorderPoints loop2c(%d:%d,%d), LonLat=(%lld,%lld)\n", length1,i,j, myMapBorderGrid1[i].lon,myMapBorderGrid1[i].lat);
    }

    int length2 = myMapBorderGrid2.size();
    printf("T1Map2-findBorderPoints step3, length(%d->%d), Min(%lld,%lld), Max(%lld,%lld)\n", length1,length2, mapMin.lon,mapMin.lat, mapMax.lon,mapMax.lat);
    if ( length2==0 ) return -1;

    x1=myMapBorderGrid2[0].lon; y1=myMapBorderGrid2[0].lat;   // Last kept point
    pointPlan.lon = mapMin.lon + meWidthGrid * ( x1-1 );
    pointPlan.lat = mapMin.lat + meWidthGrid * ( y1-1 );
    pointPlan.yaw = 0;
    myRoutePlan.clear();
    myRoutePlan.emplace_back(pointPlan);
    //printf("T1Map2-findBorderPoints loop3o(%d/%d), XY1=(%d,%d), XY2=(%d,%d), XY3=(%d,%d)\n", length2, 1, x1, y1, x2, y2, x3, y3);
    for ( int i=1; i<length2-1; i++ ) { // remove the point in same line
        //x1=myMapBorderGrid2[i-1].lon; y1=myMapBorderGrid2[i-1].lat;   // Last kept point
        x2=myMapBorderGrid2[i].lon; y2=myMapBorderGrid2[i].lat;       // Current point
        int dx = abs(x2-x1);
        int dy = abs(y2-y1);
        if ( dx<=1 && dy<=1 ) {
            while ( i<length2-1 ) {
                x2=myMapBorderGrid2[i].lon; y2=myMapBorderGrid2[i].lat;       // Current point
                x3=myMapBorderGrid2[i+1].lon; y3=myMapBorderGrid2[i+1].lat;   // Next point
                int dx2 = abs(x3-x2);
                int dy2 = abs(y3-y2);
                if ( dx2>1 || dy2>1 ) break;
                if ( (y2-y1)*(x3-x2) != (y3-y2)*(x2-x1) ) break;
                i++;
            }
        }
        //x3=myMapBorderGrid[i+1].lon; y3=myMapBorderGrid[i+1].lat;   // Next point
        //printf("T1Map2-findBorderPoints loop3(%d/%d), XY1=(%d,%d), XY2=(%d,%d), XY3=(%d,%d)\n", length, i, x1, y1, x2, y2, x3, y3);
        // Keep current point if it's not collinear with prev and next
        // Collinearity condition: (y2 - y1)*(x3 - x2) == (y3 - y2)*(x2 - x1)
        //if ( (y2-y1)*(x3-x2) == (y3-y2)*(x2-x1) ) continue;
        //long long cross = abs( (long long)(y2-y1) * (x3-x2) - (long long)(y3-y2) * (x2-x1) );
        //if ( cross <= 3 ) continue;  // this to
        //printf("T1Map2-findBorderPoints loop3a(%d/%d), XY1=(%d,%d), XY2=(%d,%d), XY3=(%d,%d)\n", length2, i, x1, y1, x2, y2, x3, y3);
        x1=x2; y1=y2;
        pointPlan.lon = mapMin.lon + meWidthGrid * ( x1-1 );
        pointPlan.lat = mapMin.lat + meWidthGrid * ( y1-1 );
        pointPlan.yaw = 0;
        myRoutePlan.emplace_back(pointPlan);
        //printf("T1Map2-findBorderPoints loop5b(%d/%d), XY1=(%d,%d), XY2=(%d,%d), XY3=(%d,%d)\n", length2, i, x1, y1, x2, y2, x3, y3);
    }
    //x1=myMapBorderGrid[length-1].lon; y1=myMapBorderGrid[length-1].lat;
    //pointPlan.lon = mapMin.lon + meWidthGrid * ( x3-1 );
    //pointPlan.lat = mapMin.lat + meWidthGrid * ( y3-1 );
    //pointPlan.yaw = 0;
    //myRoutePlan.emplace_back(pointPlan);

    int length3 = myRoutePlan.size();
    printf("T1Map2-findBorderPoints step5, length(%d->%d), Min(%lld,%lld), Max(%lld,%lld)\n", length2,length3, mapMin.lon,mapMin.lat, mapMax.lon,mapMax.lat);
    for ( int i=0; i<length3; i++ ) {
        printf("T1Map2-findBorderPoints loop5(%d/%d), XY=(%lld,%lld,%d)\n", length3, i, myRoutePlan[i].lon, myRoutePlan[i].lat, myRoutePlan[i].yaw);
    }
    printf("T1Map2-findBorderPoints end\n\n");
    return length3;
}

/* clear insid points before find points path along the border
 * 2026.05.26: creat by Leo
 *
void T1Map2::findBorderPointClear() {
    for ( int i=0; i<meGridMaxX; i++ ) {
        for ( int j=0; j<meGridMaxY; j++ ) {
            if ( myGrid[i][j]==VALUEGORBSTACLE || myGrid[i][j]==VALUEGCUT )
                myGrid[i][j] = VALUEGGRASS;
        }
    }
}*/

/* find the nearest grid inside the border point
 * 2026.05.21: created by Leo
 */
void T1Map2::findBorderPointInside(int *gx, int *gy) {
    int debug=0;
    int maxX, maxY, maxXY, begin, end, i, j;

    if ( *gx >= meGridMaxX/2 ) maxX = *gx;
    else maxX = meGridMaxX - *gx;
    if ( *gy >= meGridMaxY/2 ) maxY = *gy;
    else maxY = meGridMaxY - *gy;
    if ( maxX > maxY ) maxXY = maxX;
    else maxXY = maxY;
    if (debug>0) printf("T1Map2-findBorderPointInside begin, gxy(%d,%d), maxXY(%d,%d,%d),meGridMaxXY(%d,%d)\n",
         *gx,*gy, maxX,maxY,maxXY, meGridMaxX,meGridMaxY);
    for ( i=1; i<maxXY; i++ ) {
        if (debug>3) printf("T1Map2-findBorderPointInside LOOP(%d)\n", i);//gx,gy, maxX,maxY,maxXY, meGridMaxX,meGridMaxY);
        int line1, line2, line3, line4;
        line1 = *gy - i;       // bottom side line
        line2 = *gy + i;       // top side line
        line3 = *gx - i;       // left side line
        line4 = *gx + i;       // right side line

        begin =  line3<0 ? 0 : line3;
        end = line4>=meGridMaxX ? meGridMaxX-1 : line4;
        if ( line1 >= 0 ) { // bottom side loop
            for ( j=*gx; j>=begin; j-- ) {
                if (debug>5) printf("T1Map2-findBorderPointInside loop line1--(%d,%d)\n", j, line1);
                if ( myGrid[j][line1]>=0 || myGrid[j][line1]==VALUEGCUT ) {
                    *gx=j;
                    *gy=line1;
                    if (debug>2) printf("T1Map2-findBorderInside found line1--(%d,%d)\n", j, line1);
                    return;
                }
            }
            for ( j=*gx; j<=end; j++ ) {
                if (debug>5) printf("T1Map2-findBorderPointInside loop line1++(%d,%d)\n", j, line1);
                if ( myGrid[j][line1]>=0 || myGrid[j][line1]==VALUEGCUT ) {
                    *gx=j;
                    *gy=line1;
                    if (debug>2) printf("T1Map2-findBorderInside found line1++(%d,%d)\n", j, line1);
                    return;
                }
            }
        }
        if ( line2 < meGridMaxY ) { // top side loop
            for ( j=*gx; j>=begin; j-- ) {
                if (debug>5) printf("T1Map2-findBorderPointInside loop line2--(%d,%d)\n", j, line2);
                if ( myGrid[j][line2]>=0 || myGrid[j][line2]==VALUEGCUT ) {
                    *gx=j;
                    *gy=line2;
                    if (debug>2) printf("T1Map2-findBorderInside found line2--(%d,%d)\n", j, line2);
                    return;
                }
            }
            for ( j=*gx; j<=end; j++ ) {
                if (debug>5) printf("T1Map2-findBorderPointInside loop line2++(%d,%d)\n", j, line2);
                if ( myGrid[j][line2]>=0 || myGrid[j][line2]==VALUEGCUT ) {
                    *gx=j;
                    *gy=line2;
                    if (debug>2) printf("T1Map2-findBorderInside found line2++(%d,%d)\n", j, line2);
                    return;
                }
            }
        }
        begin =  line1<0 ? 0 : line1;
        end = line2>=meGridMaxY ? meGridMaxY-1 : line2;
        if ( line3 >= 0 ) { // left side loop
            for ( j=*gy; j>=begin; j-- ) {
                if (debug>5) printf("T1Map2-findBorderPointInside loop line3--(%d,%d)\n", line3,j);
                if ( myGrid[line3][j]>=0 || myGrid[line3][j]==VALUEGCUT ) {
                    *gx=line3;
                    *gy=j;
                    if (debug>2) printf("T1Map2-findBorderInside found line3--(%d,%d)\n", line3,j);
                    return;
                }
            }
            for ( j=*gy; j<=end; j++ ) {
                if (debug>5) printf("T1Map2-findBorderPointInside loop line3++(%d,%d)\n", line3,j);
                if ( myGrid[line3][j]>=0 || myGrid[line3][j]==VALUEGCUT ) {
                    *gx=line3;
                    *gy=j;
                    if (debug>2) printf("T1Map2-findBorderInside found line3++(%d,%d)\n", line3,j);
                    return;
                }
            }
        }
        if ( line4 < meGridMaxX ) { // right side loop
            for ( j=*gy; j>=begin; j-- ) {
                if (debug>5) printf("T1Map2-findBorderPointInside loop line4--(%d,%d)\n", line4,j);
                if ( myGrid[line4][j]>=0 || myGrid[line4][j]==VALUEGCUT ) {
                    *gx=line4;
                    *gy=j;
                    if (debug>2) printf("T1Map2-findBorderInside found line4--(%d,%d)\n", line4,j);
                    return;
                }
            }
            for ( j=*gy; j<=end; j++ ) {
                if (debug>5) printf("T1Map2-findBorderPointInside loop line4++(%d,%d)\n", line4, j);
                if ( myGrid[line4][j]>=0 || myGrid[line4][j]==VALUEGCUT ) {
                    *gx=line4;
                    *gy=j;
                    if (debug>2) printf("T1Map2-findBorderInside found line4++(%d,%d)\n", line4, j);
                    return;
                }
            }
        }
    }
    if (debug>1) printf("T1Map2-findBorderInside end, no inside grid be found, use the origin grid.\n");
}

/* Find the nearest RoutePlanLine in uncut area of grid map
 * RoutePlanLine has been marked in grids
 * Stop searching beyond obstacle in each direction
 * 2026.05.01: created by Leo(+AI)
 * 2026.05.24: modified by Leo(+AI), stop search if meet a obstacle grid in this direction
 */
int T1Map2::findNearestLine(int currX, int currY) {
    int debug=1;
    int gx, gy, maxX, maxY, maxXY, begin, end, i, j;
    int lim1L=0, lim1R=meGridMaxX;
    int lim2L=0, lim2R=meGridMaxX;
    int lim3D=0, lim3U=meGridMaxY;
    int lim4D=0, lim4U=meGridMaxY;
    gx = ( currX-mapMin.lon ) / meWidthGrid + 1;
    gy = ( currY-mapMin.lat ) / meWidthGrid + 1;
    if ( gx>=meGridMaxX/2 ) maxX = gx;
    else maxX = meGridMaxX - gx;
    if ( gy>=meGridMaxY/2 ) maxY = gy;
    else maxY = meGridMaxY - gy;
    if ( maxX>=maxY ) maxXY = maxX;
    else maxXY = maxY;
    if (debug>0) printf("T1Map2-findNearestLine begin, gxy(%d,%d), maxXY(%d,%d,%d), meGridMaxXY(%d,%d)\n", gx,gy, maxX,maxY,maxXY, meGridMaxX,meGridMaxY);
    for ( i=1; i<maxXY; i++ ) {
        if (debug>1) printf("\n==========\nT1Map2-findNearestLine LOOP(%d)\n", i);
        int line1, line2, line3, line4;
        line1 = gy - i;
        line2 = gy + i;
        line3 = gx - i;
        line4 = gx + i;
        begin =  line3<0 ? 0 : line3;
        end = line4>=meGridMaxX ? meGridMaxX-1 : line4;
        if ( line1>=0 ) {
            int b = begin < lim1L ? lim1L : begin;
            for( j=gx; j>=b; j-- ){
                if (debug>1) printf("T1Map2-findNearestLine loop line1 (%d,%d)\n", j, line1);
                if ( myGrid[j][line1] == VALUEGORBSTACLE ) { lim1L = j+1; break; }
                if ( myGrid[j][line1]>0 ) return myGrid[j][line1]-1;
            }
            int e = end > lim1R ? lim1R : end;
            for( j=gx; j<=e; j++ ){
                if (debug>1) printf("T1Map2-findNearestLine loop line1 (%d,%d)\n", j, line1);
                if ( myGrid[j][line1] == VALUEGORBSTACLE ) { lim1R = j-1; break; }
                if ( myGrid[j][line1]>0 ) return myGrid[j][line1]-1;
            }
        }
        if ( line2<meGridMaxY ) {
            int b = begin < lim2L ? lim2L : begin;
            for( j=gx; j>=b; j-- ){
                if (debug>1) printf("T1Map2-findNearestLine loop line2 (%d,%d)\n", j, line2);
                if ( myGrid[j][line2] == VALUEGORBSTACLE ) { lim2L = j+1; break; }
                if ( myGrid[j][line2]>0 ) return myGrid[j][line2]-1;
            }
            int e = end > lim2R ? lim2R : end;
            for( j=gx; j<=e; j++ ){
                if (debug>1) printf("T1Map2-findNearestLine loop line2 (%d,%d)\n", j, line2);
                if ( myGrid[j][line2] == VALUEGORBSTACLE ) { lim2R = j-1; break; }
                if ( myGrid[j][line2]>0 ) return myGrid[j][line2]-1;
            }
        }
        begin =  line1<0 ? 0 : line1;
        end = line2>=meGridMaxY ? meGridMaxY-1 : line2;
        if ( line3>=0 ) {
            int b = begin < lim3D ? lim3D : begin;
            for( j=gy; j>=b; j-- ){
                if (debug>1) printf("T1Map2-findNearestLine loop line3 (%d,%d)\n", line3, j);
                if ( myGrid[line3][j] == VALUEGORBSTACLE ) { lim3D = j+1; break; }
                if ( myGrid[line3][j]>0 ) return myGrid[line3][j]-1;
            }
            int e = end > lim3U ? lim3U : end;
            for( j=gy; j<=e; j++ ){
                if (debug>1) printf("T1Map2-findNearestLine loop line3 (%d,%d)\n", line3, j);
                if ( myGrid[line3][j] == VALUEGORBSTACLE ) { lim3U = j-1; break; }
                if ( myGrid[line3][j]>0 ) return myGrid[line3][j]-1;
            }
        }
        if ( line4<meGridMaxX ) {
            int b = begin < lim4D ? lim4D : begin;
            for( j=gy; j>=b; j-- ){
                if (debug>1) printf("T1Map2-findNearestLine loop line4 (%d,%d)\n", line4, j);
                if ( myGrid[line4][j] == VALUEGORBSTACLE ) { lim4D = j+1; break; }
                if ( myGrid[line4][j]>0 ) return myGrid[line4][j]-1;
            }
            int e = end > lim4U ? lim4U : end;
            for( j=gy; j<=e; j++ ){
                if (debug>1) printf("T1Map2-findNearestLine loop line4 (%d,%d)\n", line4, j);
                if ( myGrid[line4][j] == VALUEGORBSTACLE ) { lim4U = j-1; break; }
                if ( myGrid[line4][j]>0 ) return myGrid[line4][j]-1;
            }
        }
    }
    if (debug>1) printf("T1Map2-findNearestLine end, no line be found.\n");
    return GetLength('L');
}

/* find the nearest line with fallback to full search
 * First try four-direction search with obstacle avoidance
 * If failed, fallback to simple spiral search without obstacle limits
 * 2026.06.15: created by BY
 */
int T1Map2::findNearestLineBY(int currX, int currY) {
    int debug=1;
    int gx, gy, maxX, maxY, maxXY, begin, end, i, j;
    int lim1L=0, lim1R=meGridMaxX;
    int lim2L=0, lim2R=meGridMaxX;
    int lim3D=0, lim3U=meGridMaxY;
    int lim4D=0, lim4U=meGridMaxY;
    gx = ( currX-mapMin.lon ) / meWidthGrid + 1;
    gy = ( currY-mapMin.lat ) / meWidthGrid + 1;
    if ( gx>=meGridMaxX/2 ) maxX = gx;
    else maxX = meGridMaxX - gx;
    if ( gy>=meGridMaxY/2 ) maxY = gy;
    else maxY = meGridMaxY - gy;
    if ( maxX>=maxY ) maxXY = maxX;
    else maxXY = maxY;
    if (debug>0) printf("T1Map2-findNearestLineBY begin, gxy(%d,%d), maxXY(%d,%d,%d), meGridMaxXY(%d,%d)\n", gx,gy, maxX,maxY,maxXY, meGridMaxX,meGridMaxY);
    
    // Phase 1: Four-direction search with obstacle avoidance
    for ( i=1; i<maxXY; i++ ) {
        if (debug>1) printf("\n==========\nT1Map2-findNearestLineBY LOOP(%d)\n", i);
        int line1, line2, line3, line4;
        line1 = gy - i;
        line2 = gy + i;
        line3 = gx - i;
        line4 = gx + i;
        begin =  line3<0 ? 0 : line3;
        end = line4>=meGridMaxX ? meGridMaxX-1 : line4;
        if ( line1>=0 ) {
            int b = begin < lim1L ? lim1L : begin;
            for( j=gx; j>=b; j-- ){
                if (debug>1) printf("T1Map2-findNearestLineBY loop line1 (%d,%d)\n", j, line1);
                if ( myGrid[j][line1] == VALUEGORBSTACLE ) { lim1L = j+1; break; }
                if ( myGrid[j][line1]>0 ) return myGrid[j][line1]-1;
            }
            int e = end > lim1R ? lim1R : end;
            for( j=gx; j<=e; j++ ){
                if (debug>1) printf("T1Map2-findNearestLineBY loop line1 (%d,%d)\n", j, line1);
                if ( myGrid[j][line1] == VALUEGORBSTACLE ) { lim1R = j-1; break; }
                if ( myGrid[j][line1]>0 ) return myGrid[j][line1]-1;
            }
        }
        if ( line2<meGridMaxY ) {
            int b = begin < lim2L ? lim2L : begin;
            for( j=gx; j>=b; j-- ){
                if (debug>1) printf("T1Map2-findNearestLineBY loop line2 (%d,%d)\n", j, line2);
                if ( myGrid[j][line2] == VALUEGORBSTACLE ) { lim2L = j+1; break; }
                if ( myGrid[j][line2]>0 ) return myGrid[j][line2]-1;
            }
            int e = end > lim2R ? lim2R : end;
            for( j=gx; j<=e; j++ ){
                if (debug>1) printf("T1Map2-findNearestLineBY loop line2 (%d,%d)\n", j, line2);
                if ( myGrid[j][line2] == VALUEGORBSTACLE ) { lim2R = j-1; break; }
                if ( myGrid[j][line2]>0 ) return myGrid[j][line2]-1;
            }
        }
        begin =  line1<0 ? 0 : line1;
        end = line2>=meGridMaxY ? meGridMaxY-1 : line2;
        if ( line3>=0 ) {
            int b = begin < lim3D ? lim3D : begin;
            for( j=gy; j>=b; j-- ){
                if (debug>1) printf("T1Map2-findNearestLineBY loop line3 (%d,%d)\n", line3, j);
                if ( myGrid[line3][j] == VALUEGORBSTACLE ) { lim3D = j+1; break; }
                if ( myGrid[line3][j]>0 ) return myGrid[line3][j]-1;
            }
            int e = end > lim3U ? lim3U : end;
            for( j=gy; j<=e; j++ ){
                if (debug>1) printf("T1Map2-findNearestLineBY loop line3 (%d,%d)\n", line3, j);
                if ( myGrid[line3][j] == VALUEGORBSTACLE ) { lim3U = j-1; break; }
                if ( myGrid[line3][j]>0 ) return myGrid[line3][j]-1;
            }
        }
        if ( line4<meGridMaxX ) {
            int b = begin < lim4D ? lim4D : begin;
            for( j=gy; j>=b; j-- ){
                if (debug>1) printf("T1Map2-findNearestLineBY loop line4 (%d,%d)\n", line4, j);
                if ( myGrid[line4][j] == VALUEGORBSTACLE ) { lim4D = j+1; break; }
                if ( myGrid[line4][j]>0 ) return myGrid[line4][j]-1;
            }
            int e = end > lim4U ? lim4U : end;
            for( j=gy; j<=e; j++ ){
                if (debug>1) printf("T1Map2-findNearestLineBY loop line4 (%d,%d)\n", line4, j);
                if ( myGrid[line4][j] == VALUEGORBSTACLE ) { lim4U = j-1; break; }
                if ( myGrid[line4][j]>0 ) return myGrid[line4][j]-1;
            }
        }
    }
    
    // Phase 2: Fallback to simple spiral search without obstacle limits
    if (debug>0) printf("T1Map2-findNearestLineBY Phase1 failed, fallback to simple search\n");
    for ( i=1; i<maxXY; i++ ) {
        if (debug>1) printf("\n==========\nT1Map2-findNearestLineBY FALLBACK LOOP(%d)\n", i);
        int line1, line2, line3, line4;
        line1 = gy - i;       // bottom side line
        line2 = gy + i;       // top side line
        line3 = gx - i;       // left side line
        line4 = gx + i;       // right side line

        begin =  line3<0 ? 0 : line3;
        end = line4>=meGridMaxX ? meGridMaxX-1 : line4;
        if ( line1>=0 ) { // bottom side loop
            for ( j=begin; j<=end; j++ ) {
                if (debug>1) printf("T1Map2-findNearestLineBY fallback line1 (%d,%d)\n", j, line1);
                if ( myGrid[j][line1]>0 ) return myGrid[j][line1]-1;
            }
        }
        if ( line2<meGridMaxY ) { // top side loop
            for ( j=begin; j<=end; j++ ) {
                if (debug>1) printf("T1Map2-findNearestLineBY fallback line2 (%d,%d)\n", j, line2);
                if ( myGrid[j][line2]>0 ) return myGrid[j][line2]-1;
            }
        }
        begin =  line1<0 ? 0 : line1;
        end = line2>=meGridMaxY ? meGridMaxY-1 : line2;
        if ( line3>=0 ) { // left side loop
            for ( j=begin; j<=end; j++ ) {
                if (debug>1) printf("T1Map2-findNearestLineBY fallback line3 (%d,%d)\n", line3, j);
                if ( myGrid[line3][j] > 0 ) return myGrid[line3][j]-1;
            }
        }
        if ( line4<meGridMaxX ) { // right side loop
            for ( j=begin; j<=end; j++ ) {
                if (debug>1) printf("T1Map2-findNearestLineBY fallback line4 (%d,%d)\n", line4, j);
                if ( myGrid[line4][j]>0 ) return myGrid[line4][j]-1;
            }
        }
    }
    
    if (debug>0) printf("T1Map2-findNearestLineBY end, no line be found.\n");
    return GetLength('L');
}

/* find the nearest line of the whole RoutePlanLine in grid map's uncut area
 * the RoutePlanLine have been marked in grids
 * 2026.05.01: modifiied by Leo
 */
/*int T1Map2::findNearestLine(int currX, int currY) {
    int debug=0;
    int gx, gy, maxX, maxY, maxXY, begin, end, i, j;
    gx = ( currX-mapMin.lon ) / meWidthGrid + 1;
    gy = ( currY-mapMin.lat ) / meWidthGrid + 1;
    if ( gx>=meGridMaxX/2 ) maxX = gx;
    else maxX = meGridMaxX - gx;
    if ( gy>=meGridMaxY/2 ) maxY = gy;
    else maxY = meGridMaxY - gy;
    if ( maxX>=maxY ) maxXY = maxX;
    else maxXY = maxY;
    if (debug>0) printf("T1Map2-findNearestLine begin, gxy(%d,%d), maxXY(%d,%d,%d), meGridMaxXY(%d,%d)\n", gx,gy, maxX,maxY,maxXY, meGridMaxX,meGridMaxY);
    for ( i=1; i<maxXY; i++ ) {
        if (debug>1) printf("\n==========\nT1Map2-findNearestLine LOOP(%d)\n", i);//gx,gy, maxX,maxY,maxXY, meGridMaxX,meGridMaxY);
        int line1, line2, line3, line4;
        line1 = gy - i;       // bottom side line
        line2 = gy + i;       // top side line
        line3 = gx - i;       // left side line
        line4 = gx + i;       // right side line

        begin =  line3<0 ? 0 : line3;
        end = line4>=meGridMaxX ? meGridMaxX-1 : line4;
        if ( line1>=0 ) { // bottom side loop
            for ( j=begin; j<=end; j++ ) {
                if (debug>1) printf("T1Map2-findNearestLine loop line1 (%d,%d)\n", j, line1);//gx,gy, maxX,maxY,maxXY, meGridMaxX,meGridMaxY);
                if ( myGrid[j][line1]>0 ) return myGrid[j][line1]-1;
            }
        }
        if ( line2<meGridMaxY ) { // top side loop
            for ( j=begin; j<=end; j++ ) {
                if (debug>1) printf("T1Map2-findNearestLine loop line2 (%d,%d)\n", j, line2);//gx,gy, maxX,maxY,maxXY, meGridMaxX,meGridMaxY);
                if ( myGrid[j][line2]>0 ) return myGrid[j][line2]-1;
            }
        }
        begin =  line1<0 ? 0 : line1;
        end = line2>=meGridMaxY ? meGridMaxY-1 : line2;
        if ( line3>=0 ) { // left side loop
            for ( j=begin; j<=end; j++ ) {
                if (debug>1) printf("T1Map2-findNearestLine loop line3 (%d,%d)\n", line3, j);//gx,gy, maxX,maxY,maxXY, meGridMaxX,meGridMaxY);
                if ( myGrid[line3][j] > 0 ) return myGrid[line3][j]-1;
            }
        }
        if ( line4<meGridMaxX ) { // right side loop
            for ( j=begin; j<=end; j++ ) {
                if (debug>1) printf("T1Map2-findNearestLine loop line4 (%d,%d)\n", line4, j);//gx,gy, maxX,maxY,maxXY, meGridMaxX,meGridMaxY);
                if ( myGrid[line4][j]>0 ) return myGrid[line4][j]-1;
            }
        }
    }
    if (debug>1) printf("T1Map2-findNearestLine end, no line be found.\n");
    return GetLength('L');
}*/

/*unsigned char T1Map2::GridValueTo2Bit(int gridValue) {
    if (gridValue == VALUEGGRASS) {
        return 0b00; 
    } else if (gridValue == VALUEGCUT) {
        return 0b01;  
    } else if (gridValue == VALUEGFORB || gridValue == VALUEGBORD) {
        return 0b10;  
    } else {
        return 0b00;  
    }
}
void T1Map2::Save2BitMapToFile(const std::string& filename) {
    if (!myGrid || meGridMaxX <= 0 || meGridMaxY <= 0) {
        printf("T1Map2-Save2BitMapToFile: Grid not initialized or empty\n");
        return;
    }

    FILE* fp = fopen(filename.c_str(), "wb");
    if (!fp) {
        printf("T1Map2-Save2BitMapToFile: Failed to open file %s\n", filename.c_str());
        return;
    }

    fwrite(&meGridMaxX, sizeof(int), 1, fp);
    fwrite(&meGridMaxY, sizeof(int), 1, fp);

    int bytesPerRow = (meGridMaxX + 3) / 4;  
    unsigned char* rowBuffer = new unsigned char[bytesPerRow];

    for (int y = meGridMaxY -1; y >= 0; --y) {
        memset(rowBuffer, 0, bytesPerRow);  

        for (int x = 0; x < meGridMaxX; ++x) {
            int gridValue = myGrid[x][y];
            unsigned char twoBit = GridValueTo2Bit(gridValue);
            
            int byteIndex = x / 4;
            int bitOffset = (3 - (x % 4)) * 2; 
            
            rowBuffer[byteIndex] |= (twoBit << bitOffset);
        }

        fwrite(rowBuffer, sizeof(unsigned char), bytesPerRow, fp);
    }

    delete[] rowBuffer;
    fclose(fp);
    printf("T1Map2-Save2BitMapToFile: 2-bit map saved to %s\n", filename.c_str());
}*/
