/************************************
Silva Medina José Arnulfo
Robots Móviles y Agentes Inteligentes
Práctica 01
*************************************/

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

/**********************************

Declaracion de variables booleanas 

-go: moverse
-object: si detecta un objeto

***********************************/
bool go = true, object = false; 


//Delaracion de funciones auxiliares (callbacks)

//Callback que permite indicar si el robot se mueve o no 
void callback_start_move(const std_msgs::Bool::ConstPtr& msg)
{
	std::cout << "Date: " << (int)msg->data << std::endl; //Muestra el dato que se recibe
	go = msg->data;	//Se modifica el valor a go
}

//Callback que recibe la información del laser
void callback_laser(const sensor_msgs::LaserScan::ConstPtr& msg){

	float distance = msg->ranges[msg->ranges.size() / 2];  //Calcula la distancia que exite entre el robot y el objeto
	std::cout << "Distance = " << distance << std::endl;
	object = distance <= 0.5; // Si la distancia es menor o igual a 0.5, la variable object indicara que exite un obstaculoa esa distancia
}


int main(int argc, char** argv)
{
	std::cout << "Initializing node..." << std::endl;
	ros::init(argc, argv, "silva_medina"); //Se inicializa el nodo con el nombre del paquete
	ros::NodeHandle n; //Se comunica con el nodo master
	ros::Subscriber sub_start = n.subscribe("/start_move", 1, callback_start_move);	//Se suscribe al topico "start_move"
	ros::Subscriber sub_scan = n.subscribe("/hardware/scan", 1, callback_laser); 	//Se suscribe al topico "scan"
	ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>(//Se crea un publicador para el topico cmd_vel 
			"/hardware/mobile_base/cmd_vel", 1);
	geometry_msgs::Twist msg_cmd_vel;//Se declara la variable tipo twist para la velocidad
	ros::Rate loop(10);//Frecuencia de 10 Hz

	while(ros::ok())
	{	
		msg_cmd_vel.linear.x = 0.5; //Velocidad en x=0.5
		if(go && !object)
			pub_cmd_vel.publish(msg_cmd_vel);//Si go es verdadero y object es falso, el robot se mueve, cualquier condición
		ros::spinOnce();			 // diferente el robot se detiene
		loop.sleep();
	}
	return 0;
}
