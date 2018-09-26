#include "ros/ros.h" //Por defecto
#include "std_msgs/Bool.h"// Tipo de dato del mensaje
#include "geometry_msgs/Twist.h" // Tipo de mensaje sobre el tipo que sera 
#include "sensor_msgs/LaserScan.h"  //Biblioiteca para controlar el sensor 
bool move = true; //Variable para controlar en que momento se mueve
bool obstacle = false; //Variable para saber si en frente hay un obstaculo


void callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg)  //Esta funcion resive  las variables del sensor 
{
	//Declaracion de variable para saber en que momento detenerse 
	float front_dist = msg->ranges[msg->ranges.size() / 2];
	std::cout << "Front distance = " << front_dist << std::endl; //imprime las distancias recorridas 
	
	obstacle = front_dist < 0.5;
}


void callback_start_move(const std_msgs::Bool::ConstPtr& msg) //funcion para ll
{
	std::cout << "Received: " << (int)msg->data << std::endl; 
	move = msg->data;	
}

int main(int argc, char** argv)
{
	std::cout << "Initializing node...(Y)" << std::endl; 
	ros::init(argc, argv, "Garcia_Gomez"); 
	ros::NodeHandle n;
	
	//Crear uno subscriptor
	ros::Subscriber sub_start = n.subscribe("/start_move", 1, callback_start_move); 
	
	ros::Subscriber sub_scan = n.subscribe("/hardware/scan", 1, callback_scan);
	
	//Crear un publicador
	ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>(
			"/hardware/mobile_base/cmd_vel", 1);
	
	geometry_msgs::Twist msg_cmd_vel;
	
	ros::Rate loop(10);
	
	while(ros::ok())// ciclio para tomar los datos del publicador 
	{	
		msg_cmd_vel.linear.x = 0.5;
		if(move && !obstacle)
			pub_cmd_vel.publish(msg_cmd_vel);
		ros::spinOnce();
		loop.sleep();
	}
	return 0;
}
