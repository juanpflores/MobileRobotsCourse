#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
bool move = false;
bool obstacle = false;

void callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	float front_dist = msg->ranges[msg->ranges.size() / 2];
	std::cout << "Front distance = " << front_dist << std::endl;
	obstacle = front_dist < 0.5;
}

void callback_start_move(const std_msgs::Bool::ConstPtr& msg)
{
	std::cout << "Received: " << (int)msg->data << std::endl;
	move = msg->data;
}

int main(int argc, char** argv)
{
	std::cout << "Initializing node...(Y)" << std::endl;
	ros::init(argc, argv, "carranza_ayala"); //Tercer argumento es nombre de nodo
	ros::NodeHandle n; //El que se comunica con el rosmaster
	ros::Subscriber sub_start = n.subscribe("/start_move", 1, callback_start_move);
	ros::Subscriber sub_scan = n.subscribe("/hardware/scan", 1, callback_scan);
	ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>(
			"/hardware/mobile_base/cmd_vel", 1);
	geometry_msgs::Twist msg_cmd_vel;

	ros::Rate loop(10);

	while(ros::ok())
	{
		//Instrucciones que pueden tardar entre 10 y 90 ms
		msg_cmd_vel.linear.x = 0.5;
		if(move && !obstacle)
			pub_cmd_vel.publish(msg_cmd_vel);
		ros::spinOnce();
		loop.sleep();
	}
	return 0;
}
