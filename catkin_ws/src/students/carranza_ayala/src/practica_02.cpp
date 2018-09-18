/*
 * ROBOTS MÓVILES Y AGENTES INTELIGENTES
 * FACULTAD DE INGENIERÍA, UNAM, 2019-1
 * P R Á C T I C A   2
 * CONTROL DE POSICIÓN
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
// #define NOMBRE "APELLIDO_PATERNO_APELLIDO_MATERNO"
#define NOMBRE "CARRANZA_AYALA"

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

    //Variables para los errores
    float error_x;
    float error_y;
    float error_a;
    //Límites de velocidad
    float vmax = 0.5;
    float wmax = 0.5;
    //Constantes para las fórmulas
    float k = 0.5;
    float alpha = 0.5;
    float beta = 1.0;

    //Se calculan los errores de posición y ángulo
    error_x = goal_x - robot_x;
    error_y = goal_y - robot_y;
    error_a = goal_a - robot_a;

    //Si el error angular supera los límites de pi o -pi, se ajustan a este rango
    if(error_a < -M_PI)
        error_a += 2*M_PI;
    if (error_a > M_PI)
        error_a -= 2*M_PI;

    //Si la distancia al objetivo es menor a 0.1, se detiene el robot
    if(abs(error_x) <= 0.1 && abs(error_y) <= 0.1)
    {
        msg_cmd_vel.linear.x = 0;
        msg_cmd_vel.linear.y = 0;
        msg_cmd_vel.angular.z = 0;
    }
    //Si es mayor, se calcula la velocidad dependiendo del tipo de base
    else
    {
    if(control_type == "omni") //Para una base omnidireccional
    {
        //Se calcula la velocidad linear en X y en Y
        msg_cmd_vel.linear.x = k * error_x * cos(robot_a) + k * error_y * sin(robot_a);
        msg_cmd_vel.linear.y = -k * error_x * sin(robot_a) + k * error_y * cos(robot_a);
        //Si el cálculo supera la velocidad máxima permitida, se coloca la velocidad como la vel máxima
        if (msg_cmd_vel.linear.x > 0.5)
            msg_cmd_vel.linear.x = vmax;
        if (msg_cmd_vel.linear.y > 0.5)
            msg_cmd_vel.linear.y = vmax;
        if (msg_cmd_vel.linear.x < 0.5)
            msg_cmd_vel.linear.x = -vmax;
        if (msg_cmd_vel.linear.y < 0.5)
            msg_cmd_vel.linear.y = -vmax;
    }
    else
    {
        //Se calcula la velocidad linear y la velocidad angular según las fórmulas
        msg_cmd_vel.linear.x = vmax * exp(-pow(error_a, 2) / alpha);
        msg_cmd_vel.angular.z = wmax * (2 / (1 + exp(-(error_a / beta))) - 1);
    }
    }

	pub_cmd_vel.publish(msg_cmd_vel);
	ros::spinOnce();
	loop.sleep();
    }
    
    return 0;
    
}

