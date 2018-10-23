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
#define NOMBRE "GARCÍA_GARCÉS"

/*
 * Variables to store the current robot position and the global goal position.
 */
float global_goal_x;
float global_goal_y;
float robot_x;
float robot_y;
float robot_a;
/*
 * global_plan: stores the path to be tracked.
 * cltPlanPath: service client used to call the service for calculating a path. 
 */
nav_msgs::Path global_plan;
ros::ServiceClient clt_plan_path;


float error_global = 1;
float goal_local_x;
float goal_local_y;
bool new_req;

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

  new_req = true;
}

//Función que calculará las distancias entre dos puntos de global_plan
float dist(float a,float b, float c, float d)
{
  float d1 = abs(a-b);
  float d2 = abs(c-d);
  float distance = sqrt(d1*d1 + d2*d2);

  return distance;
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

      //Verifica que no se haya llegado al final de la ruta con error_global
      //Recorre los puntos de la ruta y cuando la distancia sea menor actualiza con un nuevo punto
      if(error_global > 0.1 && new_req==true)
	{
	  for(int i = 0; i < global_plan.poses.size()-1 ;i++)
	    {
	      float dist_local = dist(global_plan.poses[i].pose.position.x,robot_x,global_plan.poses[i].pose.position.y,robot_y);
	       if(dist_local < 0.001)
	      	{
		  goal_local_x = global_plan.poses[i+1].pose.position.x;
		  goal_local_y = global_plan.poses[i+1].pose.position.y;
	      	}
	    }
	  
	  float nx = goal_local_x;
	  float ny = goal_local_y;	  

	  //Control para robot diferencial
	  float pi = 3.14159265;
	  float error_x = nx - robot_x;
	  float error_y = ny - robot_y;
	  
	  float error_a = atan2(error_y,error_x) - robot_a;
	  float vell_max = 0.5;
	  float vela_max = 0.5;
	  float alpha = 0.6;
	  float beta = 0.3;
	  float factor_l,vel_l,vel_a;

	  if(error_a < -pi)
	    error_a += 2.0*pi;
	  if(error_a > pi)
	    error_a -= 2.0*pi;

	  factor_l = exp(-(error_a*error_a)/alpha);
	  vel_l = vell_max*factor_l;

	  vel_a = vela_max*(2/(1+exp(-error_a/beta))-1);


	  error_global = sqrt((global_goal_x-robot_x)*(global_goal_x-robot_x) + (global_goal_y-robot_y)*(global_goal_y-robot_y));
    
	  msg_cmd_vel.linear.x = vel_l;
	  msg_cmd_vel.linear.y = 0.0;
	  msg_cmd_vel.angular.z = vel_a;
	 
	}

      else{
	  msg_cmd_vel.linear.x = 0.0;
	  msg_cmd_vel.linear.y = 0.0;
	  msg_cmd_vel.angular.z = 0.0;

	  error_global = 1;
	  new_req = false;
	}
	
	       
      pub_cmd_vel.publish(msg_cmd_vel);
      ros::spinOnce();
      loop.sleep();
	}
  return 0;
}
