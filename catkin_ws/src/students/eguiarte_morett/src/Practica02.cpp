/*
 * ROBOTS MÓVILES Y AGENTES INTELIGENTES
 * FACULTAD DE INGENIERÍA, UNAM, 2019-1
 * P R Á C T I C A   2
 * CONTROL DE POSICIÓN
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"

#define NOMBRE "EGUIARTE_MORETT"

int main(int argc, char** argv)
{
    std::cout << "PRÁCTICA 02 - Control de Posición - " << NOMBRE << std::endl;
    ros::init(argc, argv, "practica_02");
    ros::NodeHandle n("~");

    if(!n.hasParam("x") || !n.hasParam("y"))
    {
    	std::cout << "Parameters \"x\" and \"y\" are required. " << std::endl;
    	return -1;
    }
    //Variables required for the position control
    float robot_x, robot_y,robot_a, goal_x, goal_y, goal_a, error_x, error_y, error_a, v_x_a, v_y_a, w, v_x, v_y, v_a, Kp = 0.5;
    std::string control_type = "";

    //Getting parameteres to set goal position and control type.
    n.param<std::string>("type", control_type, "diff"); //The default control type is for a differential base
    n.param<float>("a", goal_a, 0.0);                   //The default goal angle is 0.0 rad
    if(!n.getParam("x", goal_x))
    {
    	std::cout << "Invalid x value." << std::endl;
    	return -1;
    }
    if(!n.getParam("y", goal_y))
    {
      std::cout << "Invalid y value." << std::endl;
      return -1;
    }
    if(control_type != "diff"  && control_type != "omni")
    {
      std::cout << "Valid control types are \"omni\" and \"diff\"." << std::endl;
	    return -1;
    }

    if(control_type == "omni")
      std::cout << "Executing position control for an OMNIDIRECTIONAL base with " << std::endl;
    else
	    std::cout << "Executing position control for a DIFFERENTIAL base with " << std::endl;
    std::cout << "Goal X=" << goal_x << "\tGoal Y=" << goal_y << "\tGoal Angle=" << goal_a << std::endl;

    //Required subscriptors, publishers and messages
    ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel", 1);
    ros::Rate loop(10);
    tf::TransformListener tl;
    tf::StampedTransform t;
    tf::Quaternion q;
    geometry_msgs::Twist msg_cmd_vel;
    while(ros::ok())
    {
	     //Instructions needed to get the current robot position
	      tl.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(0.5));
	      tl.lookupTransform("map", "base_link", ros::Time(0), t);
	      robot_x = t.getOrigin().x();
	      robot_y = t.getOrigin().y();
	      q = t.getRotation();
	      robot_a = atan2(q.z(), q.w())*2;

      	/*
      	 * Calculate the values of msg_cmd_vel according to the control type.
      	 * (for omnidirectional or differential base)
      	 * Declare more variables if necessary.
      	 * The maximum linear speed must be 0.5 m/s
      	 * The maximul angular speed must be 0.5 rad/s
      	 * The robot must stop if it is at 0.1 m or less from the goal point
      	 */
         if(control_type == "omni"){ //Si control es omnidireccional
         //error de posicion: posicion deseada - posicion actual
           error_x = goal_x - robot_x;
           error_y = goal_y - robot_y;
          //error de angulo: angulo deseado -  angulo actual
           error_a = goal_a - robot_a;
          //Efectuando un control proporcional
           v_x_a = Kp * error_x;
           v_y_a = Kp * error_y;
           w = Kp *  error_a;
          //Obteniendo la velocidad lineal para ambos ejes de movimiento
           v_x = v_x_a * cos(robot_a) + v_y_a * sin(robot_a);
           v_y = -v_y_a * sin(robot_a) + v_y_a * cos(robot_a);
         }else{//Si no, el control es diferencial
          //error de posicion: posicion deseada - posicion actual
           error_x = goal_x - robot_x;
           error_y = goal_y - robot_y;
          //angulo deseado: angulo del vector de error
           goal_a = atan2(error_y,error_x);
          //error del angulo: angulo deseado - angulo actual
           error_a = goal_a - robot_a;
          //nos aseguramos que la rotacion del robot sea la minima hasta el ángulo deseado
           if(error_a < - M_PI)
              error_a+=2*M_PI;
           else if(error_a > M_PI)
              error_a-=2*M_PI;
          //si la distancia hasta el punto meta es mayor o igual que 0.1 mts calculamos la funcion de velocidad lineal y angular (gaussiana y sigmoide respectivamente)
          if(sqrt(error_x*error_x + error_y*error_y)>=0.1){
            v_a = Kp * exp(-error_a*error_a/0.5);
            w = Kp *  (2/(1 + exp(-error_a/0.5))-1);
          } //si no, el robot ha llegado al punto meta, por lo que debemos detenerlo
          else{
            v_a = 0;
            w = 0;
          }
          //Se calcula la velocidad lineal en el sistema de referencia absoluto
           v_x = v_a * cos(robot_a);
           v_y = v_a * sin(robot_a);
           
         }
        //Publicamos velocidad lineal y angular del robot
        msg_cmd_vel.linear.x = v_x;
        msg_cmd_vel.linear.y = v_y;
        msg_cmd_vel.angular.z = w;
      	pub_cmd_vel.publish(msg_cmd_vel);
      	ros::spinOnce();
      	loop.sleep();
    }

    return 0;

}
