#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
bool move = false;
bool obstacle = false;

// Funcion que recibe datos del sensor laser
void callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
        float front_dist = msg->ranges[msg->ranges.size() / 2];
        std::cout << "Front distance = " << front_dist << std::endl;
        obstacle = front_dist < 0.5;
}

// Funcion que recibe datos del movimiento de robot
void callback_start_move(const std_msgs::Bool::ConstPtr& msg)
{
        std::cout << "Received: " << (int)msg->data << std::endl;
        move = msg->data;
}

int main(int argc, char** argv)
{
        std::cout << "Initializing training node...(Y)" << std::endl;
        ros::init(argc, argv, "practica_01"); //Tercer argumento es nombre de nodo
        ros::NodeHandle n; //El que se comunica con el rosmaster
        ros::Subscriber sub_start = n.subscribe("/start_move", 1, callback_start_move); //Suscriptor para escuchar si se quier realizar un movimiento
        ros::Subscriber sub_scan = n.subscribe("/hardware/scan", 1, callback_scan); //Suscriptor para escuchar si se detecta un obstaculo
        ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>(
                        "/hardware/mobile_base/cmd_vel", 1); //Publicador del movimiento
        geometry_msgs::Twist msg_cmd_vel;

        ros::Rate loop(10);

        while(ros::ok())
        {
                //Instrucciones que pueden tardar entre 10 y 90 ms
                msg_cmd_vel.linear.x = 0.5;
		// Si no se encuentra obstaculo y se sigue publicando el movimento, mueve el robot. Sino, detendlo.
                if(move && !obstacle)
                        pub_cmd_vel.publish(msg_cmd_vel);
                ros::spinOnce();
                loop.sleep();
        }
        return 0;
}

