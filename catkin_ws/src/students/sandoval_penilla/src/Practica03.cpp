/*
 * ROBOTS MÓVILES Y AGENTES INTELIGENTES
 * FACULTAD DE INGENIERÍA, UNAM, 2019-1
 * P R Á C T I C A   3
 * PLANEACIÓN DE RUTAS MEDIANTE ALGORITMOS DE BÚSQUEDA
 */
#include "ros/ros.h"
#include "nav_msgs/GetPlan.h"
#include "nav_msgs/GetMap.h"
#include "PathPlanner.h"
#define NOMBRE "Sandoval_Penilla"

ros::ServiceClient cltGetMap;
nav_msgs::Path     msgPath;

bool callback_bfs(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res)
{
    float start_x = req.start.pose.position.x;
    float start_y = req.start.pose.position.y;
    float goal_x  = req.goal.pose.position.x;
    float goal_y  = req.goal.pose.position.y;
    std::cout << "Practica03.->Planning path by BREADTH FIRST SEARCH algorithm for:" << std::endl;
    std::cout << "Start position: X="<< start_x << "\tY=" << start_y << std::endl;
    std::cout << "Goal  position: X="<< goal_x  << "\tY=" << goal_y  <<  std::endl;
    nav_msgs::GetMap srv;
    if(!cltGetMap.call(srv))
    {
	std::cout << "Practica03.->Cannot get occupancy grid :'(" << std::endl;
	return false;
    }
    std::cout << "Map width=" << srv.response.map.info.width << "\theight=" << srv.response.map.info.height;
    std::cout << "\tResolution="<< srv.response.map.info.resolution << std::endl;
    std::cout << "Map origint: X="<< srv.response.map.info.origin.position.x;
    std::cout << "\tY=" << srv.response.map.info.origin.position.y << std::endl;

    if(!PathPlanner::BreadthFirstSearch(start_x, start_y, goal_x, goal_y, srv.response.map, res.plan))
    {
	std::cout << "Practica03.->Cannot calculate path using BREADTH FIRST SEARCH. :'(" << std::endl;
	return false;
    }
    msgPath = res.plan;
    std::cout << std::endl;
    return true;
}

bool callback_dfs(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res)
{
    float start_x = req.start.pose.position.x;
    float start_y = req.start.pose.position.y;
    float goal_x  = req.goal.pose.position.x;
    float goal_y  = req.goal.pose.position.y;
    std::cout << "Practica03.->Planning path by DEPTH FIRST SEARCH algorithm for:" << std::endl;
    std::cout << "Start position: X="<< start_x << "\tY=" << start_y << std::endl;
    std::cout << "Goal  position: X="<< goal_x  << "\tY=" << goal_y  <<  std::endl;
    nav_msgs::GetMap srv;
    if(!cltGetMap.call(srv))
    {
	std::cout << "Practica03.->Cannot get occupancy grid :'(" << std::endl;
	return false;
    }
    std::cout << "Map width=" << srv.response.map.info.width << "\theight=" << srv.response.map.info.height;
    std::cout << "\tResolution="<< srv.response.map.info.resolution << std::endl;
    std::cout << "Map origint: X="<< srv.response.map.info.origin.position.x;
    std::cout << "\tY=" << srv.response.map.info.origin.position.y << std::endl;

    if(!PathPlanner::DepthFirstSearch(start_x, start_y, goal_x, goal_y, srv.response.map, res.plan))
    {
	std::cout << "Practica03.->Cannot calculate path using DEPTH FIRST SEARCH. :'(" << std::endl;
	return false;
    }
    msgPath = res.plan;
    std::cout << std::endl;
    return true;
}

bool callback_dijkstra(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res)
{
    float start_x = req.start.pose.position.x;
    float start_y = req.start.pose.position.y;
    float goal_x  = req.goal.pose.position.x;
    float goal_y  = req.goal.pose.position.y;
    std::cout << "Practica03.->Planning path by DIJKSTRA algorithm for:" << std::endl;
    std::cout << "Start position: X="<< start_x << "\tY=" << start_y << std::endl;
    std::cout << "Goal  position: X="<< goal_x  << "\tY=" << goal_y  <<  std::endl;
    nav_msgs::GetMap srv;
    if(!cltGetMap.call(srv))
    {
	std::cout << "Practica03.->Cannot get occupancy grid :'(" << std::endl;
	return false;
    }
    std::cout << "Map width=" << srv.response.map.info.width << "\theight=" << srv.response.map.info.height;
    std::cout << "\tResolution="<< srv.response.map.info.resolution << std::endl;
    std::cout << "Map origint: X="<< srv.response.map.info.origin.position.x;
    std::cout << "\tY=" << srv.response.map.info.origin.position.y << std::endl;

    if(!PathPlanner::Dijkstra(start_x, start_y, goal_x, goal_y, srv.response.map, res.plan))
    {
	std::cout << "Practica03.->Cannot calculate path using DIJKSTRA. :'(" << std::endl;
	return false;
    }
    msgPath = res.plan;
    std::cout << std::endl;
    return true;
}

bool callback_a_star(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res)
{
    float start_x = req.start.pose.position.x;
    float start_y = req.start.pose.position.y;
    float goal_x  = req.goal.pose.position.x;
    float goal_y  = req.goal.pose.position.y;
    std::cout << "Practica03.->Planning path by A STAR algorithm for:" << std::endl;
    std::cout << "Start position: X="<< start_x << "\tY=" << start_y << std::endl;
    std::cout << "Goal  position: X="<< goal_x  << "\tY=" << goal_y  <<  std::endl;
    nav_msgs::GetMap srv;
    if(!cltGetMap.call(srv))
    {
	std::cout << "Practica03.->Cannot get occupancy grid :'(" << std::endl;
	return false;
    }
    std::cout << "Map width=" << srv.response.map.info.width << "\theight=" << srv.response.map.info.height;
    std::cout << "\tResolution="<< srv.response.map.info.resolution << std::endl;
    std::cout << "Map origint: X="<< srv.response.map.info.origin.position.x;
    std::cout << "\tY=" << srv.response.map.info.origin.position.y << std::endl;

    if(!PathPlanner::AStar(start_x, start_y, goal_x, goal_y, srv.response.map, res.plan))
    {
	std::cout << "Practica03.->Cannot calculate path using A STAR. :'(" << std::endl;
	return false;
    }
    msgPath = res.plan;
    std::cout << std::endl;
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "PRÁCTICA 03 - PLANEACIÓN DE RUTAS MEDIANTE ALGORITMOS DE BUSQUEDA - " << NOMBRE << std::endl;
    ros::init(argc, argv, "practica_03");
    ros::NodeHandle n("~");
    ros::ServiceServer srvBFS      = n.advertiseService("/navigation/path_planning/breadth_first_search", callback_bfs);
    ros::ServiceServer srvDFS      = n.advertiseService("/navigation/path_planning/depth_first_search",  callback_dfs);
    ros::ServiceServer srvDijkstra = n.advertiseService("/navigation/path_planning/dijkstra_search", callback_dijkstra);
    ros::ServiceServer srvAStar    = n.advertiseService("/navigation/path_planning/a_star_search",  callback_a_star);
    ros::Publisher     pubPath     = n.advertise<nav_msgs::Path>("/navigation/path_planning/path", 1);
    cltGetMap = n.serviceClient<nav_msgs::GetMap>("/navigation/localization/static_map");
    ros::Rate loop(5);
    
    while(ros::ok())
    {
	pubPath.publish(msgPath);
	ros::spinOnce();
	loop.sleep();
    }
}
