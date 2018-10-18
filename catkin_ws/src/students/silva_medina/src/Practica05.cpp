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
#define NOMBRE "Silva_Medina"

/*
 * Variables to store the current robot position and the global goal position.
 */
float global_goal_x;
float global_goal_y;
float robot_x;
float robot_y;
float robot_a;
int i;

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
    i=0;
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
	float goal_x, goal_y, error_a, error_x, error_y, goal_a, v_x, v_y, w, v_max = 1.0, w_max = 1.0;
	
	//Se busca el punto que tiene una distancia mayor a 0.2 para tomar como nodo meta
        while(i < global_plan.poses.size()){
            goal_x = global_plan.poses[i].pose.position.x;
            goal_y = global_plan.poses[i].pose.position.y;
	    
	    float distance=sqrt(pow(goal_x-robot_x,2)+pow(goal_y-robot_y,2));		
            if(distance > 0.2)
		break;
            i++;
        }

       //Se utilizan las ecuaciones de control usadas anteriormente para mover al robot al punto deseado

        error_x = goal_x - robot_x;
        error_y = goal_y - robot_y;
        goal_a = atan2(error_y,error_x);
        error_a = goal_a - robot_a;

        if(error_a < - M_PI)
          error_a+=2*M_PI;
        else if(error_a > M_PI)
          error_a-=2*M_PI;
        error_x = global_goal_x - robot_x; 
        error_y = global_goal_y - robot_y;
	
	float global_error=sqrt(pow(error_x,2)+pow(error_y,2));
	
        if(global_error>=0.1){
            v_x = v_max * exp(-error_a*error_a/0.5);
            w = w_max *  (2/(1 + exp(-error_a/0.5))-1);
        } 
	else { 
            v_x = 0;
            w = 0;    
        }

        //Publicamos las velocidades lineales y angulares del robot
        msg_cmd_vel.linear.x = v_x;
        msg_cmd_vel.linear.y = 0;
        msg_cmd_vel.angular.z = w;

	pub_cmd_vel.publish(msg_cmd_vel);
	ros::spinOnce();
	loop.sleep();
    }
    return 0;
}
