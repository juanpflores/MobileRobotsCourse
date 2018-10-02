/*
 * ROBOTS MÓVILES Y AGENTES INTELIGENTES
 * FACULTAD DE INGENIERÍA, UNAM, 2019-1
 * P R Á C T I C A   2
 * CONTROL DE POSICIÓN
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#define NOMBRE "Silva Medina"

int main(int argc, char** argv)
{
    std::cout << "PRÁCTICA 02 - Control de Posición - " << NOMBRE << std::endl;
    ros::init(argc, argv, "practica_02");
    ros::NodeHandle n("~");

    if(!n.hasParam("x") || !n.hasParam("y"))
    {
	std::cout << "Parameters \"x\" and \"y\" are required. " << std::endl;
	return -1;
    }
    //Variables required for the position control
    float robot_x;
    float robot_y;
    float robot_a;
    float goal_x;
    float goal_y;
    float goal_a;
    std::string control_type = "";

    //Getting parameteres to set goal position and control type.
    n.param<std::string>("type", control_type, "diff"); //The default control type is for a differential base
    n.param<float>("a", goal_a, 0.0);                   //The default goal angle is 0.0 rad
    if(!n.getParam("x", goal_x))
    {
	std::cout << "Invalid x value." << std::endl;
	return -1;
    }
    if(!n.getParam("y", goal_y))
    {
	std::cout << "Invalid y value." << std::endl;
	return -1;
    }
    if(control_type != "diff"  && control_type != "omni")
    {
	std::cout << "Valid control types are \"omni\" and \"diff\"." << std::endl;
	return -1;
    }
    
    if(control_type == "omni")
	std::cout << "Executing position control for an OMNIDIRECTIONAL base with " << std::endl;
    else
	std::cout << "Executing position control for a DIFFERENTIAL base with " << std::endl;
    std::cout << "Goal X=" << goal_x << "\tGoal Y=" << goal_y << "\tGoal Angle=" << goal_a << std::endl;

    //Required subscriptors, publishers and messages
    ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel", 1);
    ros::Rate loop(10);
    tf::TransformListener tl;
    tf::StampedTransform t;
    tf::Quaternion q;
    geometry_msgs::Twist msg_cmd_vel;

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
	//TODO:
	//Calculate the values of msg_cmd_vel according to the control type.
	//Declare more variables if necessary.
	//The maximum linear speed must be 0.5 m/s
	//The maximul angular speed must be 0.5 rad/s
	//The robot must stop if it is at 0.1 m or less from the goal point
	float alpha = 0.5;
	float beta  = 0.1;
	float v_max = 0.5;
	float w_max = 0.5;
	float Kpl   = 1.0;
	float Kpa   = 1.0;

	float error_x = goal_x - robot_x;
	float error_y = goal_y - robot_y;
	float error_a;
	float error_m = sqrt(error_x*error_x + error_y*error_y);
	if(control_type == "omni")
	    error_a = goal_a - robot_a;
	else
	    error_a = atan2(error_y, error_x) - robot_a;
	if(error_a >  M_PI) error_a -= 2*M_PI;
	if(error_a < -M_PI) error_a += 2*M_PI;

	if(control_type == "omni")
	{
	    msg_cmd_vel.linear.x  = Kpl * ( error_x * cos(robot_a) + error_y * sin(robot_a));
	    msg_cmd_vel.linear.y  = Kpl * (-error_x * sin(robot_a) + error_y * cos(robot_a));
	    msg_cmd_vel.angular.z = Kpa * error_a;
	    if(msg_cmd_vel.linear.x >  v_max) msg_cmd_vel.linear.x =  v_max;
	    if(msg_cmd_vel.linear.x < -v_max) msg_cmd_vel.linear.x = -v_max;
	    if(msg_cmd_vel.linear.y >  v_max) msg_cmd_vel.linear.y =  v_max;
	    if(msg_cmd_vel.linear.y < -v_max) msg_cmd_vel.linear.y = -v_max;
	}
	else if(error_m > 0.1)
	{
	    msg_cmd_vel.linear.x  = v_max * exp(-error_a*error_a/alpha);
	    msg_cmd_vel.linear.y  = 0;
	    msg_cmd_vel.angular.z = w_max * (2 / (1 + exp(-error_a/beta)) - 1);
	}
	else
	{
	    msg_cmd_vel.linear.x  = 0;
	    msg_cmd_vel.linear.y  = 0;
	    msg_cmd_vel.angular.z = 0;
	}

	pub_cmd_vel.publish(msg_cmd_vel);
	ros::spinOnce();
	loop.sleep();
    }
    
    return 0;
    
}
