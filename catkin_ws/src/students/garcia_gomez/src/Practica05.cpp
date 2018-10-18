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
#define NOMBRE "GARCIA GOMEZ"

float gol_par;
float gol_par2;
/*
 * Variables to store the current robot position and the global goal position.
 */
float global_goal_x;
float global_goal_y;
float robot_x;
float robot_y;
float robot_a;
int ban;

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
    ban = 2;

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
    
    float err= 1;			
    float err2= 0;
    int iter=0;
   
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
    
    float alpha = 0.6548;
    float beta  = 0.01;
    float v_max = 0.5;
    float w_max = 0.5;

   
    if(global_plan.poses.size()!=0 && ban==2){
        if(err > 0.1){
            if(err2 <= 0.08){
              
            	for(int i=iter; i<global_plan.poses.size(); i++){
        				float dis_geo = sqrt((global_plan.poses[i].pose.position.x - robot_x)*(global_plan.poses[i].pose.position.x - robot_x) 
                            + (global_plan.poses[i].pose.position.y - robot_y)*(global_plan.poses[i].pose.position.y - robot_y));
        if(dis_geo>= 0.4){
            gol_par = global_plan.poses[i].pose.position.x;
            gol_par2 = global_plan.poses[i].pose.position.y;
            iter =i;
        
        }
    }

            }else if(err<= 0.4){
              gol_par= global_plan.poses[global_plan.poses.size()-1].pose.position.x;
                gol_par2 = global_plan.poses[global_plan.poses.size()-1].pose.position.y;
            }
            float error_x = gol_par - robot_x;
            float error_y = gol_par2 - robot_y;
            float error_a = atan2(error_y, error_x) - robot_a;
            err2= sqrt(error_x*error_x + error_y*error_y);
            err = sqrt((global_goal_x-robot_x)*(global_goal_x-robot_x) + (global_goal_y-robot_y)*(global_goal_y-robot_y));
         
            if(error_a >  M_PI) error_a -= 2*M_PI;
            if(error_a < -M_PI) error_a += 2*M_PI;

            msg_cmd_vel.angular.z = w_max * (2 / (1 + exp(-error_a/beta)) - 1);
            msg_cmd_vel.linear.x  = v_max * exp(-error_a*error_a/alpha);
            msg_cmd_vel.linear.y  = 0;
            pub_cmd_vel.publish(msg_cmd_vel);
             
        
        }else{

            ban = 1;
            msg_cmd_vel.linear.x  = 0;
            msg_cmd_vel.linear.y  = 0;
            msg_cmd_vel.angular.z = 0;
            iter=0;
            err = 1;
            err2 = 0;
            pub_cmd_vel.publish(msg_cmd_vel);
            

        }
    }

	
	ros::spinOnce();
	loop.sleep();
    }
    return 0;
}
