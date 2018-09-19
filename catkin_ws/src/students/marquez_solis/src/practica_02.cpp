#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
//Bibliotecas para manejar las transformaciones
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

using tf2::Vector3;
using tf2::Transform;
using geometry_msgs::Twist;
constexpr double PI = 3.1415926535897932384626433832795028841971;

Twist omni_control(const Transform &pose, const Twist& target, double k = 0.25,double tolerance = 0.05, double turn_ratio = 0.1);

class Practica2{
    ros::Publisher pub_cmd_vel;
    ros::NodeHandle n;
    ros::NodeHandle n_private;
    ros::Rate rate;
    Twist output;
    Twist target;
    bool omni;
    //Listener de posicion
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
public:
    Practica2() 
        :rate(10),
         n_private("~"),
         tfListener(tfBuffer)
       {
           pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel",1);
           std::string tipo_arg;
           n_private.getParam("tipo",tipo_arg);
           omni = tipo_arg == "omni";
           n_private.getParam("x",target.linear.x);
           n_private.getParam("y",target.linear.y);
           n_private.getParam("w",target.angular.z);
        }
    void loop(){
        unsigned long k;
	while (ros::ok()){
		ros::spinOnce();
                k++;
                try{
                    //Dada la razonable tristeza por la cual los tipos de datos de geometry_msgs y tf2
                    //no son compatibles, hay que realizar este procedimiento
                    
                    //No se puede convertir mediante fromMsg si no tiene estampa de tiempo
                    tf2::Stamped<Transform> tf_main;
                    
                    //Puede generar excepciones si el buffer esta vacío y
                    //la nomenclatura es diferente (no se utiliza '/')
                    auto geom_tf = tfBuffer.lookupTransform("map","base_link",ros::Time(0));
                    
                    //Lo que se busca
                    tf2::fromMsg(geom_tf,tf_main);
                    
                    
                    Twist vel_target = omni_control(tf_main,target);
                    pub_cmd_vel.publish(vel_target );
                    
                    //Sólo mostrar el estado cada 8 tiempos
                    if(k % 8 == 0){
                        //std::cout << geom_tf << std::endl;
                    }
                //Ignorar excepciones (mala idea en general)
                }catch(...){
                    
                }
                rate.sleep();
	}
    }
};
int main(int argc,char** argv){
	ros::init(argc,argv,"practica_02");
        Practica2 app;
        app.loop();
	return 0;
}
double norm_angle (double angle){
    double trunc = fmod(angle,2*PI);
    if (trunc > PI) return -2*PI + trunc;
    else if (trunc < -PI) return 2*PI + trunc;
    else return trunc;
}
//Funcion de control omnidireccional
Twist omni_control(const Transform &pose, const Twist& target, double k,double tolerance, double turn_ratio){
    Twist out;
    //Obtener direccion absoluta deseada
    auto delta = Vector3(target.linear.x,target.linear.y,0) - pose.getOrigin();
    double dist_s = tf2::tf2Dot(delta,delta);
    delta.normalize();
    auto rot = pose.getRotation();
    double angle = tf2::getYaw(rot);
    //Invertir rotación
    rot[3] = -rot[3];
    //Aplicar rotacción y escalar
    auto vel_vector = tf2::quatRotate (rot,delta)*k*(1-1/(dist_s/(tolerance*tolerance)+1));
    double angle_error = norm_angle( target.angular.z - angle);
    
    std::cout << "angle " << angle << " target " << target.angular.z << "error" << angle_error << std::endl;
    out.linear = tf2::toMsg(vel_vector);
    out.linear.z = 0;
    out.angular.z = angle_error*turn_ratio*(1-1/(angle_error*angle_error*PI*PI*4 + 1));
    return out;
}