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
#define NOMBRE "rodriguez_miranda"

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
    // std::cout << "Global_plan:" <<global_plan.poses.size()<< std::endl;
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
	nav_msgs::Path newPath=global_plan;
	
	
	//Haciendo un arreglo para almacenar los valores de global_plan
	
	float ruta_x [newPath.poses.size()];
        float ruta_y [newPath.poses.size()];
	for(int i=0; i < global_plan.poses.size(); i++)
	  {
	    ruta_x[i]=newPath.poses[i].pose.position.x;
	    ruta_y[i]=newPath.poses[i].pose.position.y;
	   	    
	  }
	
	//Variables de calculo de velocidad del diferencial
         float alfa=0.5;
         float beta=0.5;
         float vmax=0.5;
         float wmax=0.5;
         float ex;
         float ey;
         float etheta;
         float v;
         float w;
	 	 	
	 for(int i=0; i<global_plan.poses.size() ; i++)
	   {				
	    ex=ruta_x[i]-robot_x;
	    ey=ruta_y[i]-robot_y;
	
            //Si el error excede cualquiera de estas condiciones el robot no avanza
	     if (ex>=0.15 | ey>=0.15 | ex<=-0.15 | ey<=-0.15)
	      {
	     
		
	       }

	    //Si no el robot pouede avanzar al siguiente punto, puesto que la tolerancia es la admitida
	    else
	      
	     {
	       etheta=atan2(ey,ex)-robot_a;
	          	 
               if( etheta<-3.141516)
	        {
	          etheta+=2*3.141516;
	        }

	       if (etheta>3.141516)
	        {
	        etheta-=2*3.141516;
	        }
	    
               v=vmax*(exp(-etheta*etheta/alfa));
	       w=wmax*(2/(1+exp(-etheta/beta))-1);

	      if(sqrt(ex*ex + ey*ey)>0.1)
	       {
	       //Asignando valores a la velocidad
	        msg_cmd_vel.linear.x=v;
	        msg_cmd_vel.linear.y=0;
	        msg_cmd_vel.angular.z=w;
	       }
	      else{
	        msg_cmd_vel.linear.x=0;
	        msg_cmd_vel.linear.y=0;
                msg_cmd_vel.angular.z=0;
	          }
	     }
	  
	   }
	    
	pub_cmd_vel.publish(msg_cmd_vel);
	ros::spinOnce();
	loop.sleep();
    }
    return 0;
}
