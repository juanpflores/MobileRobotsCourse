#pragma once
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <iostream>
#include <stack>
#include <queue>

class Node
{
public:
    Node();
    ~Node();

    int   index;           //The index of the corresponding cell in the occupancy grid.
    int   distance;        //The accumulated distance of this node.
    int   f_value;         //The f-value, used only in the A* algorithm.
    bool  in_open_list;    //A value indicating whether this node is in the open list or not.
    bool  in_closed_list;  //A value indicating whether this node is in the closed list or not.
    Node* parent;          //A pointer to the parent of this node.
    static int manhattan_distance(int x, int y, int goal_x, int goal_y){ //Manhattan Distance Heuristic for A*
        return std::abs(x - goal_x) + std::abs(y - goal_y); 
    } 
};

class CompareByDistance
{
public:
    bool operator()(Node* n1, Node* n2) { return n1->distance > n2->distance; }
};

class CompareByFValue
{
public:
    bool operator()(Node* n1, Node* n2) { return n1->f_value > n2->f_value; }
};

class PathPlanner
{
public:
    static bool BreadthFirstSearch(float start_x, float start_y, float goal_x, float goal_y,
				     nav_msgs::OccupancyGrid& map, nav_msgs::Path& result);
    static bool DepthFirstSearch(float start_x, float start_y, float goal_x, float goal_y,
				     nav_msgs::OccupancyGrid& map, nav_msgs::Path& result);
    static bool Dijkstra(float start_x, float start_y, float goal_x, float goal_y,
				     nav_msgs::OccupancyGrid& map, nav_msgs::Path& result);
    static bool AStar(float start_x, float start_y, float goal_x, float goal_y,
				     nav_msgs::OccupancyGrid& map, nav_msgs::Path& result);

private:
    enum MODE{ //Enum for "generalizing" the pathplanning algorithm 
        BFS,
        DFS,
        DIJKSTRA,
        ASTAR
    };
    /*
    * Path planning algorithm
    * this method abstracts the general form of the above path planning algorithms
    */
    static bool algorithm(float start_x, float start_y, float goal_x, float goal_y,
				     nav_msgs::OccupancyGrid& map, nav_msgs::Path& result, PathPlanner::MODE mode);

};



