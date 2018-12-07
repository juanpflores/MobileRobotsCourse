/*
 * ROBOTS MÓVILES Y AGENTES INTELIGENTES
 * FACULTAD DE INGENIERÍA, UNAM, 2019-1
 * P R Á C T I C A   4
 * SUAVIZADO DE RUTAS MEDIANTE DESCENSO DEL GRADIENTE
 */
#include "ros/ros.h"
#include "nav_msgs/GetPlan.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "PathPlanner.h"
#include "PathSmoother.h"

#define METHOD_BFS      0
#define METHOD_DFS      1
#define METHOD_DIJKSTRA 2
#define METHOD_ASTAR    3

#define NOMBRE "SANCHEZ_GONZALEZ"

float smoothing_alpha;
float smoothing_beta;
float inflation_radius;
float nearness_radius;

ros::NodeHandle*   nPtr;
ros::ServiceClient cltGetMap;
ros::Publisher     pubOriPath;
ros::Publisher     pubSmoothPath;
ros::Publisher     pubInflMap;   
ros::Publisher     pubNearMap;   
nav_msgs::Path          msgOriginalPath;
nav_msgs::Path          msgSmoothedPath;
nav_msgs::OccupancyGrid inflatedMap;
nav_msgs::OccupancyGrid nearnessMap;

void get_smoothing_parameters()
{
    nPtr->getParam("/navigation/path_planning/smoothing_alpha",  smoothing_alpha);
    nPtr->getParam("/navigation/path_planning/smoothing_beta" ,  smoothing_beta);
    nPtr->getParam("/navigation/path_planning/inflation_radius", inflation_radius);
    nPtr->getParam("/navigation/path_planning/nearness_radius",  nearness_radius);
}

bool retrieve_map(nav_msgs::OccupancyGrid& map)
{
    std::cout << "Practica04.->Getting map from map_server..." << std::endl;
    nav_msgs::GetMap srv;
    if(!cltGetMap.call(srv))
    {
        std::cout << "Practica04.->Cannot get occupancy grid :'(" << std::endl;
        return false;
    }
    map = srv.response.map;
    std::cout << "Map width="<<map.info.width<<"\theight="<<map.info.height<<"\tResolution="<<map.info.resolution<<std::endl;
    std::cout << "Map origint: X="<< map.info.origin.position.x << "\tY=" << map.info.origin.position.y << std::endl;
    return true;
}

bool generic_callback(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res, int method)
{
    nav_msgs::OccupancyGrid originalMap;
    if(!retrieve_map(originalMap))
	return false;

    get_smoothing_parameters();
    std::cout << "Practica04.->Inflating obstacles with radius " << inflation_radius << std::endl;
    inflatedMap = PathSmoother::InflateObstacles(originalMap, inflation_radius);
    pubInflMap.publish(inflatedMap);
    std::cout << "Practica04.->Getting nearness map with radius " << nearness_radius << std::endl;
    nearnessMap = PathSmoother::GetNearnessMap(inflatedMap, nearness_radius);
    pubNearMap.publish(nearnessMap);
    
    float start_x = req.start.pose.position.x;
    float start_y = req.start.pose.position.y;
    float goal_x  = req.goal.pose.position.x;
    float goal_y  = req.goal.pose.position.y;
    std::cout << "Start position: X="<< start_x << "\tY=" << start_y << std::endl;
    std::cout << "Goal  position: X="<< goal_x  << "\tY=" << goal_y  <<  std::endl;

    switch(method)
    {
    case METHOD_BFS:
	std::cout << "Practica04.->Planning path by BREADTH FIRST SEARCH algorithm for:" << std::endl;
	if(!PathPlanner::BreadthFirstSearch(start_x, start_y, goal_x, goal_y, nearnessMap, msgOriginalPath))
	{
	    msgOriginalPath.poses.clear();
	    msgSmoothedPath.poses.clear();
	    pubOriPath.publish(msgOriginalPath);
	    pubSmoothPath.publish(msgSmoothedPath);
	    std::cout << "Practica04.->Cannot calculate path using BREADTH FIRST SEARCH. :'(" << std::endl;
	    return false;
	}
	break;
    case METHOD_DFS:
	std::cout << "Practica04.->Planning path by DEPTH FIRST SEARCH algorithm for:" << std::endl;
	if(!PathPlanner::DepthFirstSearch(start_x, start_y, goal_x, goal_y, nearnessMap, msgOriginalPath))
	{
	    msgOriginalPath.poses.clear();
	    msgSmoothedPath.poses.clear();
	    pubOriPath.publish(msgOriginalPath);
	    pubSmoothPath.publish(msgSmoothedPath);
	    std::cout << "Practica04.->Cannot calculate path using DEPTH FIRST SEARCH. :'(" << std::endl;
	    return false;
	}
	break;
    case METHOD_DIJKSTRA:
	std::cout << "Practica04.->Planning path by DIJKSTRA algorithm for:" << std::endl;
	if(!PathPlanner::Dijkstra(start_x, start_y, goal_x, goal_y, nearnessMap, msgOriginalPath))
	{
	    msgOriginalPath.poses.clear();
	    msgSmoothedPath.poses.clear();
	    pubOriPath.publish(msgOriginalPath);
	    pubSmoothPath.publish(msgSmoothedPath);
	    std::cout << "Practica04.->Cannot calculate path using DIJKSTRA. :'(" << std::endl;
	    return false;
	}
	break;
    case METHOD_ASTAR:
	std::cout << "Practica04.->Planning path by A STAR algorithm for:" << std::endl;
	if(!PathPlanner::AStar(start_x, start_y, goal_x, goal_y, nearnessMap, msgOriginalPath))
	{
	    msgOriginalPath.poses.clear();
	    msgSmoothedPath.poses.clear();
	    pubOriPath.publish(msgOriginalPath);
	    pubSmoothPath.publish(msgSmoothedPath);
	    std::cout << "Practica04.->Cannot calculate path using A Star. :'(" << std::endl;
	    return false;
	}
	break;
    }
    pubOriPath.publish(msgOriginalPath);

    std::cout << "Practica04.->Smoothing path with parameters alpha="<<smoothing_alpha<<" and beta="<<smoothing_beta << std::endl;
    msgSmoothedPath = PathSmoother::SmoothPath(msgOriginalPath, smoothing_alpha, smoothing_beta);
    std::cout << "Practica04.->Path smoothed with parameters alpha="<<smoothing_alpha<<" and beta="<<smoothing_beta << std::endl;
    pubSmoothPath.publish(msgSmoothedPath);

    res.plan = msgSmoothedPath;
    std::cout << std::endl;
    return true;
}

bool callback_bfs(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res)
{
    return generic_callback(req, res, METHOD_BFS);
}

bool callback_dfs(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res)
{
    return generic_callback(req, res, METHOD_DFS);
}

bool callback_dijkstra(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res)
{
    return generic_callback(req, res, METHOD_DIJKSTRA);
}

bool callback_a_star(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res)
{
    return generic_callback(req, res, METHOD_ASTAR);
}

int main(int argc, char** argv)
{
    std::cout << "PRÁCTICA 04 - SUAVIZADO DE RUTAS MEDIANTE DESCENSO DEL GRADIENTE - " << NOMBRE << std::endl;
    ros::init(argc, argv, "practica_04_solved");
    ros::NodeHandle n("~");
    nPtr = &n;

    n.param<float>("/navigation/path_planning/smoothing_alpha",  smoothing_alpha, 0.9);
    n.param<float>("/navigation/path_planning/smoothing_beta" ,  smoothing_beta,  0.01);
    n.param<float>("/navigation/path_planning/inflation_radius", inflation_radius,   0.3);
    n.param<float>("/navigation/path_planning/nearness_radius",  nearness_radius,   1.0);
    
    ros::ServiceServer srvBFS       = n.advertiseService("/navigation/path_planning/breadth_first_search", callback_bfs);
    ros::ServiceServer srvDFS       = n.advertiseService("/navigation/path_planning/depth_first_search",  callback_dfs);
    ros::ServiceServer srvDijkstra  = n.advertiseService("/navigation/path_planning/dijkstra_search", callback_dijkstra);
    ros::ServiceServer srvAStar     = n.advertiseService("/navigation/path_planning/a_star_search",  callback_a_star);
    pubOriPath   = n.advertise<nav_msgs::Path>("/navigation/path_planning/original_path", 1);
    pubSmoothPath= n.advertise<nav_msgs::Path>("/navigation/path_planning/smoothed_path", 1);
    pubInflMap   = n.advertise<nav_msgs::OccupancyGrid>("/navigation/path_planning/inflated_map", 1);
    pubNearMap   = n.advertise<nav_msgs::OccupancyGrid>("/navigation/path_planning/nearness_map", 1);
    
    cltGetMap = n.serviceClient<nav_msgs::GetMap>("/navigation/localization/static_map");
    ros::Rate loop(5);
    
    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
}
