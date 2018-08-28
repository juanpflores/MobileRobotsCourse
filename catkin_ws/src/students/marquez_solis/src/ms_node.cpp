#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
class Context{
	ros::Subscriber subs_forward;
	ros::Subscriber subs_scan;
public:
	bool fwd;
	bool obstacle;
	void forward_callback(const std_msgs::Bool::ConstPtr& bit){
		std::cout << (int)bit->data<<std::endl;
		fwd = bit->data;
	}
	void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan){
		auto middle = scan->ranges[scan->ranges.size()/2];
		obstacle = middle < 0.5;
	}
	void subscribe_to(ros::NodeHandle &n){
		subs_forward = n.subscribe<std_msgs::Bool>("/forward",2,&Context::forward_callback,this);
		subs_scan = n.subscribe<sensor_msgs::LaserScan>("/hardware/scan",2,&Context::laser_callback,this);
	}
};
int main(int argc,char** argv){
	Context q;
	ros::init(argc,argv,"ms_node");
	ros::NodeHandle n;
	q.subscribe_to(n);
	ros::Rate rate(10);
	ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel",1);
	geometry_msgs::Twist twist_stop;
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
