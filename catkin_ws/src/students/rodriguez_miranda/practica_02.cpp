/*
 * ROBOTS MÓVILES Y AGENTES INTELIGENTES
 * FACULTAD DE INGENIERÍA, UNAM, 2019-1
 * P R Á C T I C A   2
 * CONTROL DE POSICIÓN
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#define NOMBRE "rodriguez_miranda"

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
      //Para el diferencial
    if(control_type=="diff")
      {
	//Variables de calculo de velocidad del diferencial
        float alfa=3;
	float vmax=0.5;
	float ex=goal_x-robot_x;
	float ey=goal_y-robot_y;
	float etheta=atan2(ey,ex)-robot_a;
        float v;
	if( etheta<-3.141516)
	  {
	    etheta+=2*3.141516;
	  }
	if (etheta>3.141516)
	  {     
	      etheta+=2*3.141516;
	  }

	v=vmax*(exp(-etheta*etheta/alfa));

	  //Asignando valores a la velocidad
	msg_cmd_vel.linear.x=v;
	msg_cmd_vel.linear.y=v;
	//Condición para que deje de publicar
	if(goal_x!=robot_x && goal_y!=robot_y)
	   pub_cmd_vel.publish(msg_cmd_vel);
         
      }
    //Para el omnidireccional
    if(control_type=="omni") //para que lo ejecute cuando es el omni
      {
	//Variables necesarias par el omni
	float exomni=goal_x-robot_x;
	float eyomni=goal_y-robot_y;
	float ethetaomni=goal_a-robot_a;
	float vx=2*exomni;
	float vy=2*eyomni;
        if( ethetaomni<-3.141516)
	  ethetaomni+=2*3.141516;
	if(ethetaomni>3.141516)
	  ethetaomni+=2*3.141516;
	
	float vxomni=vx*cos(robot_a)+vy*sin(robot_a);
        float vyomni=-vx*sin(robot_a)+vy*cos(robot_a);
	//	float w= 2*ethetaomni;
	//Cuando llega al punto deja de pulicar
	if(goal_x!=robot_x && goal_y!=robot_y)
	  {
	    msg_cmd_vel.linear.x=vxomni;
	     msg_cmd_vel.linear.y=vyomni;
	  
	    pub_cmd_vel.publish(msg_cmd_vel);
	  }
	
      }
	ros::spinOnce();
	loop.sleep();
    }
    
    return 0;
    
}

