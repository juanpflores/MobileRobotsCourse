/*
 * ROBOTS MÓVILES Y AGENTES INTELIGENTES
 * FACULTAD DE INGENIERÍA, UNAM, 2019-1
 * P R Á C T I C A   2
 * CONTROL DE POSICIÓN
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#define NOMBRE "GARCIA_GARCES"

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

      /*
       * TODO:
       *
       * Calculate the values of msg_cmd_vel according to the control type.
       * (for omnidirectional or differential base)
       * Declare more variables if necessary.
       * The maximum linear speed must be 0.5 m/s
       * The maximul angular speed must be 0.5 rad/s
       * The robot must stop if it is at 0.1 m or less from the goal point
       */

      //
      //Variables para ambos tipos de robot
      float pi = 3.14159265;
      float error_x = goal_x - robot_x;
      float error_y = goal_y - robot_y;

      //Control para robot diferencial
      if(control_type == "diff"){
	float error_a = atan2(error_y,error_x) - robot_a;
	float vell_max = 0.5;
	float vela_max = 0.5;
	float alpha = 0.5;
	float beta = 0.5;
	float factor_l,vel_l,vel_a;

	if(error_a < -pi)
	  error_a += 2.0*pi;
	if(error_a > pi)
	  error_a -= 2.0*pi;

	factor_l = exp(-(error_a*error_a)/alpha);
	vel_l = vell_max*factor_l;

	vel_a = vela_max*(2/(1+exp(-error_a/beta))-1);

	if(sqrt(error_x*error_x + error_y*error_y) > 0.1)
	  {
	    msg_cmd_vel.linear.x = vel_l;
     	    msg_cmd_vel.linear.y = 0.0;
	    msg_cmd_vel.angular.z = vel_a;
	  }
	else{
	    msg_cmd_vel.linear.x = 0.0;
	    msg_cmd_vel.linear.y = 0.0;
	    msg_cmd_vel.angular.z = 0.0;
	  }
      }
      
      
      //Control para robot omnidireccional
      if(control_type == "omni"){
	float error_a = goal_a - robot_a;
	float kd = 1.0;
	float ka = 1.0;

	float vel_x = kd*error_x*cos(robot_a) + kd*error_y*sin(robot_a);
	float vel_y = -kd*error_x*sin(robot_a) + kd*error_y*cos(robot_a);
	float vel_a = ka*error_a;

	if(vel_x > 0.5)
	  vel_x = 0.5;
	if(vel_x < -0.5)
	  vel_x = -0.5;
       	if(vel_y > 0.5)
	  vel_y = 0.5;
	if(vel_y < -0.5)
	  vel_y = -0.5;
      	if(vel_a > 0.5)
	  vel_a = 0.5;
	if(vel_a < -0.5)
	  vel_a = -0.5;

	msg_cmd_vel.linear.x = vel_x;
	msg_cmd_vel.linear.y = vel_y;
	msg_cmd_vel.angular.z = vel_a;
      }
      

      pub_cmd_vel.publish(msg_cmd_vel);
      ros::spinOnce();
      loop.sleep();
    }

  return 0;

}
