#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
bool move = false; //Variable para controlar en que momento se mueve
bool obstacle = false; //Variable para saber si en frente hay un obstaculo

//Callback cuando se recibe informacion del laser
void callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//Se obtiene la lectura del laser en frente del robot y se imprime en consola
	float front_dist = msg->ranges[msg->ranges.size() / 2];
	std::cout << "Front distance = " << front_dist << std::endl;
	//Se determina si hay un obstaculo si la distancia que detecta el laser es menor a 0.5
	obstacle = front_dist < 0.5;
}

//Callback para controlar si se mueve o no el robot
void callback_start_move(const std_msgs::Bool::ConstPtr& msg)
{
	std::cout << "Received: " << (int)msg->data << std::endl; //Imprime el dato recibido
	move = msg->data;	//Se asigna a la variable booleana move
}

int main(int argc, char** argv)
{
	std::cout << "Initializing node...(Y)" << std::endl;
	ros::init(argc, argv, "carranza_ayala"); //Se inicializa el nodo con el nombre del paquete
	ros::NodeHandle n; //Handler para comunicarse con el nodo master
	//Se suscribe al topico "start_move" con una cola de tamaño 1
	ros::Subscriber sub_start = n.subscribe("/start_move", 1, callback_start_move);
	//Se suscribe al topico "scan" con una cola de tamaño 1
	ros::Subscriber sub_scan = n.subscribe("/hardware/scan", 1, callback_scan);
	//Se crea un publicador para el topico cmd_vel con un mensaje tipo geometry_msgs/Twist
	ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>(
			"/hardware/mobile_base/cmd_vel", 1);
	//Se declara la variable tipo twist para publicar la velocidad
	geometry_msgs::Twist msg_cmd_vel;
	//Se asigna una frecuencia de 10 Hz aproximadamente
	ros::Rate loop(10);
	//Instrucciones a ejecutar mientras el nodo esté corriendo
	while(ros::ok())
	{	//Instrucciones que pueden tardar entre 10 y 90 ms
		msg_cmd_vel.linear.x = 0.5; //Se asigna el valor de la velocidad linear en x
		//Si la variable para moverse es verdadera y no hay un obstaculo en frente
		// se publica la velocidad
		if(move && !obstacle)
			pub_cmd_vel.publish(msg_cmd_vel);
		ros::spinOnce();
		loop.sleep();
	}
	return 0;
}
