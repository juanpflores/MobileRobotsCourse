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
#define NOMBRE "Sandoval_Penilla"

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
bool new_path;


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
}

geometry_msgs::Twist calculate_speeds_diff(float goal_x,float goal_y){
    float alpha = 1;
    float beta  = 0.1;
    float vMax  = 0.5;
    float wMax  = 0.5;
    float errX  = goal_x - robot_x;
    float errY  = goal_y - robot_y;
    float errA  = atan2(errY,errX) - robot_a;

    if(errA > M_PI) errA -= 2*M_PI;
    if(errA < M_PI) errA += 2*M_PI;

    geometry_msgs::Twist msg_cmd_vel;
    msg_cmd_vel.linear.x = vMax * exp(-errA*errA/alpha);
    msg_cmd_vel.linear.y = 0;
    msg_cmd_vel.linear.z = wMax * (2/(1+exp(-errA/beta))-1);

    return msg_cmd_vel;
}

void get_next_goal(float& goal_x,float& goal_y, int& next_pose_idx, tf::TransformListener& tfListener){
    float err;
    if(next_pose_idx >= global_plan.poses.size()) next_pose_idx = global_plan.poses.size()-1;
    
    do{
        goal_x = global_plan.poses[next_pose_idx].pose.position.x;
        goal_y = global_plan.poses[next_pose_idx].pose.position.y;
        err = sqrt((goal_x-robot_x)*(goal_x-robot_x)+(goal_y-robot_y)*(goal_y-robot_y));


    }while(err < 0.25 && ++next_pose_idx < global_plan.poses.size());

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

	int next_pose_idx = 0;
    bool moving = false;				

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
	 * Write the code necessary to follow the path stored in 'global_plan'.
	 * Use the position control for a DIFFERENTIAL base.
	 * Store the linear and angular speed in msg_cmd_vel.
	 */
    


    if(new_path){
        new_path = false;
        next_pose_idx = 0;
        moving = true;
    }
    if(moving){
        float local_goal_x;
        float local_goal_y;
        get_next_goal(local_goal_x, local_goal_y, next_pose_idx,tl);
        float dist_to_goal = sqrt((global_goal_x - robot_x)*(global_goal_x-robot_x)+(global_goal_y-robot_y)*(global_goal_y-robot_y));
        pub_cmd_vel.publish(calculate_speeds_diff(local_goal_x,local_goal_y));
        if(dist_to_goal < 0.2 || new_path) moving = false;
    }

	//pub_cmd_vel.publish(msg_cmd_vel);
	ros::spinOnce();
	loop.sleep();
    }
    return 0;
}




