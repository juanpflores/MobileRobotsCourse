#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

bool move = false;//Bandera de movimiento
bool obstacle = false;//Bandera de obstaculo

/*
 * Funcion 1
 * callback que "escucha" la distancia del sensor
 * es decir recibe mensajes del topico hardware/scan
*/
void callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg){
    float front_dist = msg->ranges[msg->ranges.size() / 2]; //distancia de sensor de enmedio
    std::cout << "Front distance = " << front_dist << std::endl; //imprime distancia frontal
    obstacle = front_dist < 0.5; //obstaculo, si distancia es menor que medio metro
}

/*
 * Funcion 2
 * callback que recibe mensajes del topico /start_move
*/
void callback_start_move(const std_msgs::Bool::ConstPtr& msg){
    std::cout << "Received: " << (int)msg->data << std::endl;//imprime dato del mensaje (0 o 1) 
    move = msg->data;//asigna a la bandera move el booleano de std_msgs (mensaje)
}

/**
 * Funcion principal
 * capaz de recibir argumentos y archivos
 */
int main(int argc, char** argv){
    std::cout << "Initializing training node ... (Y)" << std::endl;
    ros::init(argc, argv, "training");//inicia ros para poder usar las funciones
    ros::NodeHandle n;//punto de acceso principal a la comunicacion, inicializa el nodo
    ros::Subscriber sub_start = n.subscribe("/start_move", 1, callback_start_move);//asigna suscriptor a topico /start_move y suscribe el nodo a este
    ros::Subscriber sub_scan = n.subscribe("/hardware/scan", 1, callback_scan);//asigna suscriptor a topico /hardware/scan y suscribe el nodo a este
    ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel", 1);//crea publicador del topico /hardware/mobile_base/cmd_vel es decir, crea publicador de velocidad
    geometry_msgs::Twist msg_cmd_vel; //mensaje que contendra vector de velocidad
    ros::Rate loop(10); //publica y escucha 10 veces por segundo

    while(ros::ok()){
        msg_cmd_vel.linear.x = 0.5; //velocidad lineal frontal de 0.5
        if(move && !obstacle) //si esta en movimiento y no hay obstaculo
            pub_cmd_vel.publish(msg_cmd_vel); //publica velocidad anteriormente asignada
        else{//si no publica velocidad de cero
            msg_cmd_vel.linear.x = 0;
            pub_cmd_vel.publish(msg_cmd_vel);
        }
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}