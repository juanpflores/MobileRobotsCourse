#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
using namespace std;

//Variabales booleanas para saber cuando mover el robot y si hay obstaculo
bool move = true;
bool obstacle = false;

/*
	Callback que se usa para el control del movimiento del robot
	el valor se muestra en consola y la variable cambia acorde a
	los valores recibidos
*/
void callback_start_move(const std_msgs::Bool::ConstPtr& msg){

	cout<<"Recibido: "<< (int)msg->data <<endl;
	move = msg->data;
}
/*
	Callback para obtener las lecturas del laser
	Con la informacion se calcula la distancia frontal a un obstacula 
	si la distancia frontal es menor a 0.5 m la variable boolena cambia de valor
*/
void callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg){
	float front_dist = msg->ranges[msg->ranges.size()/2];
	cout<<"Distancia frontal = " << front_dist <<endl;
	obstacle = front_dist < 0.5;
}

int main(int argc, char** argv){
	cout<<"Iniciando nodo..."<<endl; //mensaje de inicialiacion de nodo
	ros::init(argc,argv,"jimenez_jimenez"); //se inicializa nodo con nombre de paquete
	ros::NodeHandle n; //manejador para comunicacion con nodo master
	//subscipcion al topico start_move con cola de tamaño 1
	ros::Subscriber sub_start = n.subscribe("/start_move",1,callback_start_move);
	//subscripcion al topico scan con cola de tamaño 1
	ros::Subscriber sub_scan = n.subscribe("hardware/scan",1,callback_scan);
	//publicador de velocidad
	ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel", 1);
	//variable de tipo twist para publicar la velocidad
	geometry_msgs::Twist msg_cmd_vel;
	//frecuencia de aprox 10 hz
	ros::Rate loop(10);
	while(ros::ok()){
		msg_cmd_vel.linear.x = 0.5;//accionacion de velocidad a la var tipo twist, solo en x
		if(move && !obstacle) //condicion para la publicacion de la velocidad, solo si no hay obstaculo
			pub_cmd_vel.publish(msg_cmd_vel);//publicacion de velocidad
		ros::spinOnce();
		loop.sleep();

	}
	return 0;
}