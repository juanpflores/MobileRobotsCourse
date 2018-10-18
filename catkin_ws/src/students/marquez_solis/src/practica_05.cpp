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
#include "futils.h"
#define NOMBRE "MARQUEZ_SOLIS"

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

geometry_msgs::Twist diff_control(const tf::Transform &pose, double targetX, double targetY, double k,double tolerance, double turn_ratio, double turn_tol){
    geometry_msgs::Twist out;  
    //Dirección hacia el objetivo
    tf::Vector3 delta(targetX,targetY,0);
    delta -= pose.getOrigin();
    //Distancia manhatttan
    double dist_s = std::abs( delta[0]) + std::abs(delta[1]);
    //Cuaternion de orientacion del robot
    auto rot = pose.getRotation();
    //Angulo del robot
    double angle = tf::getYaw(rot);
    //Error de angulo
    double angle_error = norm_angle( atan2(delta[1],delta[0]) - angle);
    //El resultado es siempre un vector en X de magnitud dependiente del error del angulo y tolerancia
    out.linear.x = k*raised_cosine_norm(angle_error/turn_tol)*(1-raised_cosine_norm(dist_s/tolerance));
    //  std::cout << "angle " << angle << " target " <<  atan2(delta[1],delta[0]) << "error" << angle_error << std::endl;
    out.linear.y = 0;
    out.linear.z = 0;
    
    out.angular.z = turn_ratio*single_sine_norm(angle_error/turn_tol);
    return out;
}


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

int main(int argc, char** argv)
{
    std::cout << "PRÁCTICA 05 - SEGUIMIENTO DE RUTAS - " << NOMBRE << std::endl;
    ros::init(argc, argv, "practica_05");
    ros::NodeHandle n("~");
    ros::Rate loop(20);

    ros::Subscriber sub_goto_xya = n.subscribe("/navigation/go_to_xya", 1, callback_go_to_xya);
    ros::Publisher  pub_cmd_vel  = n.advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel", 1);
    clt_plan_path = n.serviceClient<nav_msgs::GetPlan>("/navigation/path_planning/breadth_first_search");
    tf::TransformListener tl;
    tf::StampedTransform t;
    tf::Quaternion q;
    double g_x, g_y;
    int idx = 0;
    geometry_msgs::Twist msg_cmd_vel;
    constexpr double changetol = 0.25;

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
        if(global_plan.poses.size() == 0){
            ros::spinOnce();
            loop.sleep();
            continue;
        }
        g_x = global_plan.poses[idx].pose.position.x;
        g_y =global_plan.poses[idx].pose.position.y;
        double dist_local = (g_x-robot_x)*(g_x-robot_x) + (g_y-robot_y)*(g_x-robot_y);
        std::cout << "dist: " << dist_local << std::endl;
        if( dist_local < changetol && idx < global_plan.poses.size()-1){
            idx++;
        }
        
	/*
	 * TODO:
	 * Write the code necessary to follow the path stored in 'global_path'.
	 * Use the position control for a DIFFERENTIAL base.
	 * Store the linear and angular speed in msg_cmd_vel.
	 */
        
        msg_cmd_vel = diff_control(t,g_x,g_y,0.25,0.05,0.5,0.1);
	pub_cmd_vel.publish(msg_cmd_vel);
	ros::spinOnce();
	loop.sleep();
    }
    return 0;
}