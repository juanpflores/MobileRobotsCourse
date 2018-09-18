/*
 * ROBOTS MÓVILES Y AGENTES INTELIGENTES
 * FACULTAD DE INGENIERÍA, UNAM, 2019-1
 * P R Á C T I C A   2
 * CONTROL DE POSICIÓN
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#include <math.h>
#define NOMBRE "JIMENEZ_JIMENEZ"
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
    //Variables required for the position control
    float robot_x;
    float robot_y;
    float robot_a;
    float goal_x;
    float goal_y;
    float goal_a;
    std::string control_type = "";
    //Variables para el calculo del error y velocidad
    float err_theta; //error de angulo para control diferencial
    float er_t; //error de angulo para control omnidireccional
    float vel_linear; 
    float vel_ang;
    float alpha=10.0, beta=0.01;//constantes para formulas de velocidad lineal y angular


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
    	robot_x = t.getOrigin().x();//posicion real en x del robot
    	robot_y = t.getOrigin().y(); //posicion real en y del robot
    	q = t.getRotation();
    	robot_a = atan2(q.z(), q.w())*2; //angulo real del robot

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
        
        if(control_type=="omni"){//si cumple ejecuta control omnidireccional 
            /*formulas para el calulo de la velocidad lineal en X y Y */
            msg_cmd_vel.linear.x = 0.5*(goal_x - robot_x);
            msg_cmd_vel.linear.y = 0.5*(goal_y - robot_y);
            er_t = goal_a - robot_a; //calulo de error en el angulo
            /*condiciones para girar al sentido mas proximo pa alcanzar el angulo deseado*/
            if(er_t < -PI) 
                er_t += (2*PI);
            else if(er_t > PI)
                er_t -= (2*PI);
            msg_cmd_vel.angular.z = 0.5*(goal_a - robot_a);
        
           
        }else{//sino se ejecuta el control diferencial

            err_theta = atan2((goal_y - robot_y),(goal_x - robot_x)) - robot_a;//caculo del error en el angulo
            /*condiciones para girar al sentido mas proximo pa alcanzar el angulo deseado*/
             if(err_theta < -PI) 
                err_theta += (2*PI);
            else if(err_theta > PI)
                err_theta -= (2*PI);
            /*formulas para el calculo de velocidad lineal y angular vistas en clase*/
            vel_linear = 0.5*exp(-(pow(err_theta,2)/alpha));
            vel_ang = 0.5*((2/(1+exp(-err_theta/beta)))-1);

            /*accionacion de velocidad a la var tipo twist, en X y Y, y para vel angular en z*/
            msg_cmd_vel.linear.x = vel_linear*cos(err_theta);
            msg_cmd_vel.linear.y = vel_linear*sin(err_theta);
            msg_cmd_vel.angular.z = vel_ang;

        }
        /*hace el absoluto de la resta de posicion en x deseada 
        menos la posicion en x actual*/
        float cmp;
        if((goal_x - robot_x)<0)
            cmp = -(goal_x - robot_x);
        else
            cmp = (goal_x - robot_x);
        if(!(cmp<= 0.1)) //condicion de publicacion de velocidad
            pub_cmd_vel.publish(msg_cmd_vel); 
    	ros::spinOnce();
    	loop.sleep();
    }
    
    return 0;
    
}

