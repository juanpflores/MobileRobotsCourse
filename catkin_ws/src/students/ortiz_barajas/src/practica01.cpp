//Biliotecas necesarias para utilizar ROS
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

//Booleanos para determinar si el robot debe moverse y si existe un obstáculo frente a él
bool move = false;
bool obstacle = false; 

//Callback para manejar información del lasér
void callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg){
	//Se obtiene la distancia de frente al robot, y se determina si existe un obstáculo en caso de que sea menor a 0.5[m] 
	float front_dist = msg->ranges[msg->ranges.size() / 2];
	std::cout << "Front distance = " << front_dist << std::endl;
	obstacle = front_dist < 0.5;
}

//Callback para manejar el movimiento del robot
void callback_start_move(const std_msgs::Bool::ConstPtr& msg){
	//Se imprimen los datos recibidos y se asigna al booleano move, que controla el movimiento del robot	
	std::cout << "Received: " << (int)msg->data << std::endl; 
	move = msg->data;	
}

int main(int argc, char** argv){
	std::cout << "Initializing node...(Y)" << std::endl;
	ros::init(argc, argv, "ortiz_barajas"); //Inicialización del nodo
	ros::NodeHandle n;
	//Subscriptor al tópico start_move
	ros::Subscriber sub_start = n.subscribe("/start_move", 1, callback_start_move);

	//Subscritor al tópico scan
	ros::Subscriber sub_scan = n.subscribe("/hardware/scan", 1, callback_scan);

	//Publicador para cmd_vel
	ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>(
			"/hardware/mobile_base/cmd_vel", 1);

	//Variable para publicar velocidad
	geometry_msgs::Twist msg_cmd_vel;

	ros::Rate loop(10);
	
	//Instrucciones a ejecutar mientras en nodo esté activo
	while(ros::ok()){	
		//Se asigna el valor a la componente en x de la velocidad lineal
		//Si move es verdadera y no hay obstáculos el robot se mueve
		msg_cmd_vel.linear.x = 0.5; 
		if(move && !obstacle)
			pub_cmd_vel.publish(msg_cmd_vel);
			ros::spinOnce();
			loop.sleep();
	}
	return 0;
}
