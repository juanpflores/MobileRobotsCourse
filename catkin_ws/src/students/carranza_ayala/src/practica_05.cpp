/*
 * ROBOTS MÓVILES Y AGENTES INTELIGENTES
 * FACULTAD DE INGENIERÍA, UNAM, 2019-1
 * P R Á C T I C A   5
 * SEGUIMIENTO DE RUTAS
 */

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/GetPlan.h"
#include "tf/transform_listener.h"
#define NOMBRE "CARRANZA_AYALA"

float goal_x;
float goal_y;
float robot_x;
float robot_y;
float robot_a;
bool  new_path;
nav_msgs::Path path;
ros::ServiceClient cltPlanPath;

geometry_msgs::Twist calculate_speeds_omni(float local_goal_x, float local_goal_y)
{
    float error_x = local_goal_x - robot_x;
    float error_y = local_goal_y - robot_y;
    float error_a = 0;
    float v_max = 0.5;
    float w_max = 0.5;
    float Kpl   = 1.0;
    float Kpa   = 1.0;
    geometry_msgs::Twist msg_cmd_vel;

    msg_cmd_vel.linear.x  = Kpl * ( error_x * cos(robot_a) + error_y * sin(robot_a));
    msg_cmd_vel.linear.y  = Kpl * (-error_x * sin(robot_a) + error_y * cos(robot_a));
    msg_cmd_vel.angular.z = Kpa * error_a;
    if(msg_cmd_vel.linear.x >  v_max) msg_cmd_vel.linear.x =  v_max;
    if(msg_cmd_vel.linear.x < -v_max) msg_cmd_vel.linear.x = -v_max;
    if(msg_cmd_vel.linear.y >  v_max) msg_cmd_vel.linear.y =  v_max;
    if(msg_cmd_vel.linear.y < -v_max) msg_cmd_vel.linear.y = -v_max;
    if(msg_cmd_vel.angular.z >  w_max) msg_cmd_vel.angular.z =  w_max;
    if(msg_cmd_vel.angular.z < -w_max) msg_cmd_vel.angular.z = -w_max;

    return msg_cmd_vel;
}

geometry_msgs::Twist calculate_speeds_diff(float local_goal_x, float local_goal_y)
{
    float alpha = 0.6548;
    float beta  = 0.1;
    float v_max = 0.5;
    float w_max = 0.5;
    float error_x = local_goal_x - robot_x;
    float error_y = local_goal_y - robot_y;
    float error_a = atan2(error_y, error_x) - robot_a;
    geometry_msgs::Twist msg_cmd_vel;
    if(error_a >  M_PI) error_a -= 2*M_PI;
    if(error_a < -M_PI) error_a += 2*M_PI;

    msg_cmd_vel.linear.x  = v_max * exp(-error_a*error_a/alpha);
    msg_cmd_vel.linear.y  = 0;
    msg_cmd_vel.angular.z = w_max * (2 / (1 + exp(-error_a/beta)) - 1);

    return msg_cmd_vel;
}

void get_next_goal_from_path(float& goal_x, float& goal_y, int& next_pose_idx, tf::TransformListener& tf_listener)
{
    if(next_pose_idx >= path.poses.size()) next_pose_idx = path.poses.size() - 1;
    float error = 0;
    do
    {
        goal_x = path.poses[next_pose_idx].pose.position.x;
        goal_y = path.poses[next_pose_idx].pose.position.y;
        error = sqrt((goal_x - robot_x)*(goal_x - robot_x) + (goal_y - robot_y)*(goal_y - robot_y));
    }while(error < 0.25 && ++next_pose_idx < path.poses.size());
}


void callback_go_to_xya(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    goal_x = msg->data[0];
    goal_y = msg->data[1];
    std::cout << "Practica05.->Global goal point received: X=" << goal_x << "\tY=" << goal_y << std::endl;

    std::cout << "Practica05.->Calling service for path planning..." << std::endl;
    nav_msgs::GetPlan srv;
    srv.request.start.pose.position.x = robot_x;
    srv.request.start.pose.position.y = robot_y;
    srv.request.goal.pose.position.x  = goal_x;
    srv.request.goal.pose.position.y  = goal_y;
    cltPlanPath.call(srv);
    path = srv.response.plan;
    new_path = true;
}

int main(int argc, char** argv)
{
    std::cout << "PRÁCTICA 05 - SEGUIMIENTO DE RUTAS - " << NOMBRE << std::endl;
    ros::init(argc, argv, "practica_05");
    ros::NodeHandle n("~");
    ros::Rate loop(20);

    ros::Subscriber subGoToXYA = n.subscribe("/navigation/go_to_xya", 1, callback_go_to_xya);
    ros::Publisher  pubCmdVel  = n.advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel", 1);
    cltPlanPath = n.serviceClient<nav_msgs::GetPlan>("/navigation/path_planning/a_star_search");
    tf::TransformListener tl;
    tf::StampedTransform t;
    tf::Quaternion q;

    int next_pose_idx = 0;
    bool moving = false;
    while(ros::ok())
    {
	//Instructions needed to get the current robot position
	tl.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(0.5));
	tl.lookupTransform("map", "base_link", ros::Time(0), t);
	robot_x = t.getOrigin().x();
	robot_y = t.getOrigin().y();
	q = t.getRotation();
	robot_a = atan2(q.z(), q.w())*2;
	//

	if(new_path)
	{
	    new_path = false;
	    next_pose_idx = 0;
	    moving = true;
	}
	if(moving)
	{
	    float local_goal_x;
	    float local_goal_y;
	    get_next_goal_from_path(local_goal_x, local_goal_y, next_pose_idx, tl);
	    float dist_to_goal = sqrt((goal_x - robot_x)*(goal_x - robot_x) + (goal_y - robot_y)*(goal_y - robot_y));
	    pubCmdVel.publish(calculate_speeds_diff(local_goal_x, local_goal_y));
	    if(dist_to_goal < 0.2 || new_path)
		moving = false;
	}
	
	ros::spinOnce();
	loop.sleep();
    }
    return 0;
}
