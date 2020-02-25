#pragma once
/* system interface headers */
#include <vector>
#include <utility>


/* interface header */
#include "LocalPlayer.h"

/* local interface headers */
#include "Region.h"
#include "RegionPriorityQueue.h"
#include "ServerLink.h"

#define ROW 200
#define COL 200

// Creating a shortcut for int, int pair type 
typedef std::pair<int, int> Pair;

// Creating a shortcut for pair<int, pair<int, int>> type 
typedef std::pair<double, std::pair<int, int>> pPair;

// A structure to hold the neccesary parameters 
struct cell
{
    // Row and Column index of its parent 
    // Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1 
    int parent_i, parent_j;
    // f = g + h 
    double f, g, h;
};

class Astar
{
private:
    // A Utility Function to check whether given cell (row, col) is a valid cell or not. 
    bool isValid(int row, int col);
    // A Utility Function to check whether the given cell is blocked or not 
    bool isUnBlocked(int grid[][COL], int row, int col);
    // A Utility Function to check whether destination cell has been reached or not
    bool isDestination(int row, int col, Pair dest);
    // A Utility Function to calculate the 'h' heuristics. 
    double calculateHValue(int row, int col, Pair dest);
    // A Utility Function to trace the path from the source to destination 
    float *tracePath(cell cellDetails[][COL], Pair dest);
    //Initialize the array for map
    void initializeArray(int grid[ROW][COL]);
    int floatToInt(float floatX);
    float intToFloat(int intX);
public:
    // A Function to find the shortest path between a given source cell to a destination cell according to A* Search Algorithm 
    float* aStarSearch(int grid[][COL], Pair src, Pair dest);
    // Main function to run A*
    float* runAStar(float src[2], float dest[2]);
};
