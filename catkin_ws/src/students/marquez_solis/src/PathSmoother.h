#pragma once
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <iostream>
#include <cmath>
#include <vector>

class PathSmoother
{
public:
    static nav_msgs::OccupancyGrid InflateObstacles(nav_msgs::OccupancyGrid& map,  float inflation_radius);
    static nav_msgs::OccupancyGrid GetNearnessMap(nav_msgs::OccupancyGrid& map, float nearness_radius);
    static nav_msgs::Path          SmoothPath(nav_msgs::Path& path, float alpha, float beta);           
};
