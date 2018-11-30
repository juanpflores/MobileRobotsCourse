/*
 * ROBOTS MÓVILES Y AGENTES INTELIGENTES
 * FACULTAD DE INGENIERÍA, UNAM, 2019-1
 * P R Á C T I C A   5
 * SEGUIMIENTO DE RUTAS
 */

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/GetPlan.h"
#include "tf/transform_listener.h"
#define NOMBRE "ORTIZ_BARAJAS"
# define PI 3.14159265358979323846 //Definición de PI para ajuste del angulo del robot diferencial

/*
 * Variables to store the current robot position and the global goal position.
 */
float global_goal_x;
float global_goal_y;
float robot_x;
float robot_y;
float robot_a;
/*
 * Variables for the position control with differential base
 */
float local_goal_x;
float local_goal_y;
float error_x;
float error_y;
float global_error_x;
float global_error_y;
float error_a;
float vMax = 0.5;
float wMax = 0.5;
float w;
float v;
float distance;
float i;
/*
 * global_plan: stores the path to be tracked.
 * cltPlanPath: service client used to call the service for calculating a path. 
 */
nav_msgs::Path global_plan;
ros::ServiceClient clt_plan_path;


void callback_go_to_xya(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    /*
     * Callback function. Any time a new goal xya is received, the goal coordinates and the path are
     * stored in the corresponding variables.
     */
    global_goal_x = msg->data[0];
    global_goal_y = msg->data[1];
    std::cout << "Practica05.->Global goal point received: X=" << global_goal_x << "\tY=" << global_goal_y << std::endl;

    std::cout << "Practica05.->Calling service for path planning..." << std::endl;
    nav_msgs::GetPlan srv;
    srv.request.start.pose.position.x = robot_x;
    srv.request.start.pose.position.y = robot_y;
    srv.request.goal.pose.position.x  = global_goal_x;
    srv.request.goal.pose.position.y  = global_goal_y;
    clt_plan_path.call(srv);
    global_plan = srv.response.plan;
    i = 1;
}

int main(int argc, char** argv)
{
    std::cout << "PRÁCTICA 05 - SEGUIMIENTO DE RUTAS - " << NOMBRE << std::endl;
    ros::init(argc, argv, "practica_05");
    ros::NodeHandle n("~");
    ros::Rate loop(20);

    ros::Subscriber sub_goto_xya = n.subscribe("/navigation/go_to_xya", 1, callback_go_to_xya);
    ros::Publisher  pub_cmd_vel  = n.advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel", 1);
    clt_plan_path = n.serviceClient<nav_msgs::GetPlan>("/navigation/path_planning/a_star_search");
    tf::TransformListener tl;
    tf::StampedTransform t;
    tf::Quaternion q;

    geometry_msgs::Twist msg_cmd_vel;
					

    while(ros::ok())
    {
	/*
	 * Instructions needed to get the current robot position.
	 */
	tl.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(0.5));
	tl.lookupTransform("map", "base_link", ros::Time(0), t);
	robot_x = t.getOrigin().x();
	robot_y = t.getOrigin().y();
	q = t.getRotation();
	robot_a = atan2(q.z(), q.w())*2;

	/*
	 * TODO:
	 * Write the code necessary to follow the path stored in 'global_path'.
	 * Use the position control for a DIFFERENTIAL base.
	 * Store the linear and angular speed in msg_cmd_vel.
	 */
	//Determina si se ha recibido la ruta y si la variable i no es mayor al tamaño de la ruta	
	 if(global_plan.poses.size() != NULL && i < global_plan.poses.size()){
		//Asignamos a el punto meta local a alcanzar
		local_goal_x =global_plan.poses[i].pose.position.x;
		local_goal_y =global_plan.poses[i].pose.position.y;
		//Control diferencial
		error_x = local_goal_x - robot_x;
	 	error_y = local_goal_y - robot_y;
		error_a = atan2(error_y,error_x) - robot_a;
	 	if(error_a < -PI){
			error_a += 2*PI;
	 	}
	 	if(error_a > PI){
			error_a -= 2*PI;
	 	}				
	 	w = wMax * ( (2 / (1+ exp(-error_a/1)) ) -1);
	 	v = vMax * ( exp(-pow(error_a,2)/0.5) );
	 	msg_cmd_vel.angular.z = w;
	 	msg_cmd_vel.linear.x = v;
	 	msg_cmd_vel.linear.y = 0;
	 	//Distancia euclidiana para determinar si se debe pasar al siguiente punto meta local
	 	distance = sqrt( (pow(error_y,2) - pow(error_x,2)) );
	 	//Si el robot está a menos de 0.1 de distancia del punto meta local, incrementa i para acceder al siguiente elemento		
	 	if(distance<0.1){
			i++;
	 	}
	}else{
		//Si ha recorrido todos los puntos de la ruta, se detiene
		msg_cmd_vel.angular.z = 0;
	 	msg_cmd_vel.linear.x = 0;
	 	msg_cmd_vel.linear.y = 0;		
	}
	 
	pub_cmd_vel.publish(msg_cmd_vel);
	ros::spinOnce();
	loop.sleep();
    }
    return 0;
}