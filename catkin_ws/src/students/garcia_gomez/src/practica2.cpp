/*
 * ROBOTS MÓVILES Y AGENTES INTELIGENTES
 * FACULTAD DE INGENIERÍA, UNAM, 2019-1
 * P R Á C T I C A   2
 * CONTROL DE POSICIÓN
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#define NOMBRE "García Gómez"
#define PI 3.1415926535897

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

    //variables de error
    float e_ang;
    float e_omn;
    float v_lineal;
    float v_ang;
    float a=10.0, b=0.01;
    //Variables required for the position control
    float robot_x;
    float robot_y;
    float robot_a;
    float goal_x;
    float goal_y;
    float goal_a;
    float entorno;

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

	//condicion para calcular la velocidad lineal
	if(control_type="omni"){
		msg_cmd_vel.linear.x = 0.5*(goal_x - robot_x);
		msg_cmd_vel.linear.y = 0.5*(goal_y - robot_y);
		e_omn=goal_a -robot_a;
//Condicion para poder hacer los giros correctamente
		if (e_omn<-PI)
			e_omn+=(2*PI);
		else if(e_omn>PI)
			e_omn-=(2*PI);

		msg_cmd_vel.angular.z=0.5*(goal_a -robot_a);
	}
	else{
		e_ang= atan2((goal_y - robot_y),(goal_x - robot_x))-robot_a;
		//Condicion para realizar los giros correctamente sin dar toda la vuelta completa
		if(e_ang<-PI)
			e_ang+=(2*PI);
		else if(e_ang>PI)
			e_ang-=(2*PI);
		
		v_lineal=0.5*exp(-(pow(e_ang,2)/a));
		v_ang= 0.5*((2/(1+exp(-e_ang/b)))-1);
//para accionar la velocidad 
		msg_cmd_vel.linear.x = v_lineal*cos(e_ang);
        msg_cmd_vel.linear.y = v_lineal*sin(e_ang);
        msg_cmd_vel.angular.z = v_ang;



	}

	//Para generar condiciones de publicacion en la velocidad
	    if((goal_x - robot_x)<0)
            entorno = -(goal_x - robot_x);
        else
            entorno = (goal_x - robot_x);
        if(!(entorno<= 0.1)) 
            pub_cmd_vel.publish(msg_cmd_vel); 

	pub_cmd_vel.publish(msg_cmd_vel);
	ros::spinOnce();
	loop.sleep();
    }
    
    return 0;
    
}

Practica02.cpp
Mostrando Practica02.cpp.