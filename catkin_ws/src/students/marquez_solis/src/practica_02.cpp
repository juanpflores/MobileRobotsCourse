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

Twist omni_control(const Transform &pose, const Twist& target, double k = 0.25,double tolerance = 0.05, double turn_ratio = 0.5, double turn_tol = 0.1);
Twist diff_control(const Transform &pose, const Twist& target, double k = 0.25,double tolerance = 0.05, double turn_ratio = 0.5, double turn_tol = 0.1);
//Distribucion coseno para intervalos de tolerancia, principal ventaja: es una funcion compacta
inline double raised_cosine_norm(double arg){
    return std::abs(arg) < 1 ? 0.5 + 0.5*cos(arg*PI) : 0; 
}
inline double single_sine_norm(double arg){
    if (arg < -1) return -1;
    if (arg > 1) return 1;
    auto a = sin(arg*PI/2);
    return a*a*a;
}

class Practica2{
    ros::Publisher pub_cmd_vel;
    ros::NodeHandle n;
    ros::Rate rate;
    Twist output;
    Twist target;
    bool omni;
    bool ready ;
    //Listener de posicion
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
public:
    Practica2() 
        :rate(10),
         n("~"),
         ready(false),
         tfListener(tfBuffer)
       {
           pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel",1);
        }
    bool getParams(){
        if(!n.hasParam("x") || !n.hasParam("y"))
        {
            std::cerr << "Parameters \"x\" and \"y\" are required. " << std::endl;
            return false;
        }
        std::string tipo_arg;
        n.getParam("type",tipo_arg);
        omni = tipo_arg == "omni";
        n.getParam("w",target.angular.z);
        ready = n.getParam("x",target.linear.x) &&
           n.getParam("y",target.linear.y);
        return ready;
    }
    void loop(){
        unsigned long k;
        if(!ready) return;
	while (ros::ok()){
		ros::spinOnce();
                k++;
                try{
                    //No se puede convertir mediante fromMsg si no tiene estampa de tiempo
                    tf2::Stamped<Transform> tf_main;
                    
                    auto geom_tf = tfBuffer.lookupTransform("map","base_link",ros::Time(0),ros::Duration(2.0));
                    
                    //Lo que se busca
                    tf2::fromMsg(geom_tf,tf_main);
                    Twist vel_out;
                    if (omni)
                        vel_out = omni_control(tf_main,target);
                    else
                        vel_out = diff_control(tf_main,target);
                    pub_cmd_vel.publish(vel_out );
                    
                    //Sólo mostrar el estado cada 8 tiempos
                    if(k % 8 == 0){
                        std::cout << geom_tf << std::endl;
                    }
                //Ignorar excepciones (mala idea en general)
                }catch(tf2::TransformException e){
                    
                }
                rate.sleep();
	}
    }
};
int main(int argc,char** argv){
	ros::init(argc,argv,"practica_02");
        Practica2 app;
        if(!app.getParams()){
            std::cerr << "Error Invalid arguments." << std::endl;
            return -1;
        }
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
Twist omni_control(const Transform &pose, const Twist& target, double k,double tolerance, double turn_ratio, double turn_tol){
    Twist out;
    //Obtener direccion absoluta deseada
    auto delta = Vector3(target.linear.x,target.linear.y,0) - pose.getOrigin();
    double dist_s = tf2::tf2Dot(delta,delta);
    delta.normalize();
    auto rot = pose.getRotation();
    double angle = tf2::getYaw(rot);
    //Invertir rotación
    rot[3] = -rot[3];
    //Aplicar rotacción y escalar por velocidad e intervalo de tolerancia
    auto vel_vector = tf2::quatRotate (rot,delta)*k*(1-1/(dist_s/(tolerance*tolerance)+1));
    double angle_error = norm_angle( target.angular.z - angle);
    
//    std::cout << "angle " << angle << " target " << target.angular.z << "error" << angle_error << std::endl;
    out.linear = tf2::toMsg(vel_vector);
    out.linear.z = 0;
    //Multiplicar dirección de giro por tasa de giro e intervalo de tolerancia
    out.angular.z = (std::signbit(angle_error)?-1:1)*turn_ratio*(1-raised_cosine_norm(angle_error/turn_tol));
    return out;
}
Twist diff_control(const Transform &pose, const Twist& target, double k,double tolerance, double turn_ratio, double turn_tol){
    Twist out;
    //Dirección hacia el objetivo
    auto delta = Vector3(target.linear.x,target.linear.y,0) - pose.getOrigin();
    //Distancia manhatttan
    double dist_s = std::abs( delta[0]) + std::abs(delta[1]);
    //Cuaternion de orientacion del robot
    auto rot = pose.getRotation();
    //Angulo del robot
    double angle = tf2::getYaw(rot);
    //Error de angulo
    double angle_error = norm_angle( atan2(delta[1],delta[0]) - angle);
    //El resultado es siempre un vector en X de magnitud dependiente del error del angulo y tolerancia
    auto vel_vector = Vector3(k,0,0)*raised_cosine_norm(angle_error/turn_tol)*(1-raised_cosine_norm(dist_s/tolerance));
    //  std::cout << "angle " << angle << " target " <<  atan2(delta[1],delta[0]) << "error" << angle_error << std::endl;
    out.linear = tf2::toMsg(vel_vector);
    out.linear.z = 0;
    
    out.angular.z = turn_ratio*single_sine_norm(angle_error/turn_tol);
    return out;
}