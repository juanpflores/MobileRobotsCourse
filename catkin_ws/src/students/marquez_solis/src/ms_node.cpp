#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
//Estado del robot
class Context{
	//Subscriptores
	ros::Subscriber subs_forward;
	ros::Subscriber subs_scan;
public:
	bool fwd;
	bool obstacle;
	//Callback de accion hacia adelante
	void forward_callback(const std_msgs::Bool::ConstPtr& bit){
		//std::cout << (int)bit->data<<std::endl;
		fwd = bit->data;
	}
	//Callback de scan
	void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan){
		auto middle = scan->ranges[scan->ranges.size()/2];
		obstacle = middle < 0.5;
	}
	//Registrar los subscriptores
	void subscribe_to(ros::NodeHandle &n){
		subs_forward = n.subscribe<std_msgs::Bool>("/forward",2,&Context::forward_callback,this);
		subs_scan = n.subscribe<sensor_msgs::LaserScan>("/hardware/scan",2,&Context::laser_callback,this);
	}
};
int main(int argc,char** argv){
	ros::init(argc,argv,"ms_node");
	ros::NodeHandle n;
	ros::Rate rate(10);
	Context q;
	q.subscribe_to(n);
	ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel",1);
	//Velocidad de detener
	geometry_msgs::Twist twist_stop;
	//Velociad de adelante
	geometry_msgs::Twist twist_forward;
	twist_forward.linear.x = 0.2;
	while (ros::ok()){
		if(q.fwd && !q.obstacle)
			pub_cmd_vel.publish(twist_forward);
		else
			pub_cmd_vel.publish(twist_stop);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
