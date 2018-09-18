#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

using tf2::Vector3;
using tf2::Transform;
using geometry_msgs::Twist;
//Estado del robot
Twist omni_control(const Transform &pose, const Twist& target,double tolerance = 0.05, double turn_ratio = 1, double k= 0.05){
    Twist out;
    //Obtener direccion absoluta deseada
    auto delta = (Vector3(target.linear.x,target.linear.y,0) - pose.getOrigin()).normalize();
    auto rot = pose.getRotation();
    //Invertir rotación
    rot[3] = -rot[3];
    auto vel_vector = tf2::quatRotate (rot,delta)*k;
    out.linear = tf2::toMsg(vel_vector);
    return out;
}
class Practica2{
    ros::Publisher pub_cmd_vel;
    ros::NodeHandle n;
    ros::Rate rate;
    Twist output;
    Twist target;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
public:
    Practica2() 
        :rate(10),
         tfListener(tfBuffer)
       {
           target.linear.x = 3;
           target.linear.y = 6;
           pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel",1);
        }
    void loop(){
        unsigned long k;
	while (ros::ok()){
		ros::spinOnce();
                k++;
                try{
                    tf2::Stamped<Transform> tf_main;
                    auto geom_tf = tfBuffer.lookupTransform("map","base_link",ros::Time(0));
                    tf2::fromMsg(geom_tf,tf_main);
                    Twist vel_target = omni_control(tf_main,target);
                    pub_cmd_vel.publish(vel_target );
                    if(k & 3){
                        std::cout << geom_tf << std::endl;
                    }
                }catch(...){
                    
                }
                rate.sleep();
	}
    }
};
int main(int argc,char** argv){
	ros::init(argc,argv,"practica_02");
	//Velocidad de detener
        Practica2 app;
        app.loop();
	return 0;
}
