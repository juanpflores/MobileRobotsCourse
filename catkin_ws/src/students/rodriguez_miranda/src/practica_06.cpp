/*
 * ROBOTS MÓVILES Y AGENTES INTELIGENTES
 * FACULTAD DE INGENIERÍA, UNAM, 2019-1
 * P R Á C T I C A   6
 * EVASIÓN DE OBSTÁCULOS MEDIANTE CAMPOS POTENCIALES ARTIFICIALES
 */

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"
#define NOMBRE "rodriguez_miranda"

/*
 * Variables to store the current robot position and the global goal position.
 */
float goal_x;
float goal_y;
float robot_x;
float robot_y;
float robot_a;
/*
 * Variables to store the components of the attraction, rejection and resulting forces.
 */
float attraction_x = 1.0;
float attraction_y = 0;
float rejection_x = 0;
float rejection_y = 1.0;
float resulting_x = 1;
float resulting_y = 1;

/*
 * Variables to store the design parameters of the potential fields method
 */
float Krej;
float Katt;
float d0;


void callback_go_to_xya(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    /*
     * Callback function. Any time a new goal xy is received, the goal coordinates are
     * stored in the corresponding variables.
     */
    goal_x = msg->data[0];
    goal_y = msg->data[1];
    std::cout << "Practica06.->Global goal point received: X=" << goal_x << "\tY=" << goal_y << std::endl;
    
}

void calculate_rejection_force(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    /*
     * Callback function for the laser readings. Any time a new set of readings is received, 
     * the rejection force is calculated and its components are stored in the corresponding variables.
     */
    float angle;
    float mag;
    rejection_x = 0;
    rejection_y = 0;
    for(int i=0; i < msg->ranges.size(); i++)
    {
	if (msg->ranges[i] < d0)
	    mag = Krej * sqrt(1/msg->ranges[i] - 1/d0);
	else
	    mag = 0;
	angle = msg->angle_min + i*msg->angle_increment;
	rejection_x += -mag * cos(angle + robot_a);
	rejection_y += -mag * sin(angle + robot_a);
    }
    rejection_x /= msg->ranges.size();
    rejection_y /= msg->ranges.size();
}

void calculate_attraction_force()
{
    /*
     * Function to calculate the attraction force given the robot and goal positions,
     * both stored in the corresponding global variables. 
     */
    attraction_x = goal_x - robot_x;
    attraction_y = goal_y - robot_y;
    float mag = sqrt(attraction_x*attraction_x + attraction_y*attraction_y);
    attraction_x = mag > 0 ? attraction_x / mag : 0;
    attraction_y = mag > 0 ? attraction_y / mag : 0;
    attraction_x *= Katt;
    attraction_y *= Katt;
}

void calculate_resulting_force()
{
    resulting_x = attraction_x + rejection_x;
    resulting_y = attraction_y + rejection_y;
        
}

visualization_msgs::Marker get_rejection_arrow()
{
    visualization_msgs::Marker mrk;
    geometry_msgs::Point start;
    geometry_msgs::Point end;
    start.x = robot_x;
    start.y = robot_y;
    end.x = robot_x + rejection_x;
    end.y = robot_y + rejection_y;
    mrk.header.frame_id = "map";
    mrk.ns = "pot_fields";
    mrk.id = 0;
    mrk.type = visualization_msgs::Marker::ARROW;
    mrk.action = visualization_msgs::Marker::ADD;
    mrk.scale.x = 0.05;
    mrk.scale.y = 0.07;
    mrk.color.a = 1.0;
    mrk.color.r = 1.0;
    mrk.color.g = 0;
    mrk.color.b = 0;
    mrk.points.push_back(start);
    mrk.points.push_back(end);
    return mrk;
}

visualization_msgs::Marker get_attraction_arrow()
{
    visualization_msgs::Marker mrk;
    geometry_msgs::Point start;
    geometry_msgs::Point end;
    start.x = robot_x;
    start.y = robot_y;
    end.x = robot_x + attraction_x;
    end.y = robot_y + attraction_y;
    mrk.header.frame_id = "map";
    mrk.ns = "pot_fields";
    mrk.id = 1;
    mrk.type = visualization_msgs::Marker::ARROW;
    mrk.action = visualization_msgs::Marker::ADD;
    mrk.scale.x = 0.05;
    mrk.scale.y = 0.07;
    mrk.color.a = 1.0;
    mrk.color.r = 0;
    mrk.color.g = 1.0;
    mrk.color.b = 0;
    mrk.points.push_back(start);
    mrk.points.push_back(end);
    return mrk;
}

visualization_msgs::Marker get_resulting_arrow()
{
    visualization_msgs::Marker mrk;
    geometry_msgs::Point start;
    geometry_msgs::Point end;
    start.x = robot_x;
    start.y = robot_y;
    end.x = robot_x + resulting_x;
    end.y = robot_y + resulting_y;
    mrk.header.frame_id = "map";
    mrk.ns = "pot_fields";
    mrk.id = 2;
    mrk.type = visualization_msgs::Marker::ARROW;
    mrk.action = visualization_msgs::Marker::ADD;
    mrk.scale.x = 0.05;
    mrk.scale.y = 0.07;
    mrk.color.a = 1.0;
    mrk.color.r = 0;
    mrk.color.g = 0;
    mrk.color.b = 1.0;
    mrk.points.push_back(start);
    mrk.points.push_back(end);
    return mrk;
}

int main(int argc, char** argv)
{
    std::cout << "PRÁCTICA 06 - CAMPOS POTENCIALES - " << NOMBRE << std::endl;
    ros::init(argc, argv, "practica_06");
    ros::NodeHandle n("~");
    ros::Rate loop(20);

    ros::Subscriber sub_goto_xya = n.subscribe("/navigation/go_to_xya", 1, callback_go_to_xya);
    ros::Subscriber sub_scan     = n.subscribe("/hardware/scan", 1, calculate_rejection_force);
    ros::Publisher  pub_cmd_vel  = n.advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel", 1);
    ros::Publisher  pub_markers  = n.advertise<visualization_msgs::Marker>("/hri/visualization_marker", 1);

    /*
     * T O D O :
     * These are the default parameters. After tunning the potential fields, change
     * this parameters to the ones you found.
     */
    n.param<float>("k_rej", Krej, 3.0);
    n.param<float>("k_att", Katt, 1.0);
    n.param<float>("d0", d0, 1.0);

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
	 * Call functions to calculate attraction and resulting forces. 
	 * Rejection force is calculated any time a new set of laser readings arrives. 
	 */
	calculate_attraction_force();
	calculate_resulting_force();
	
	/*
	 * T O D O :
	 * Write the code necessary to follow the path stored in 'global_path'.
	 * Use the position control for a DIFFERENTIAL base.
	 * Store the linear and angular speed in msg_cmd_vel.
	 */

        float ruta_x;
	float ruta_y;
	float uniresulx;
	float uniresuly;
	//Obteniendo el vector unitario
	 uniresulx=resulting_x/(sqrt((resulting_x*resulting_x)+(resulting_y*resulting_y)));
	 uniresuly=resulting_y/(sqrt((resulting_x*resulting_x)+(resulting_y*resulting_y)));
	 //sumandolo a la posicion del robot
	 ruta_x=0.3*uniresulx+robot_x;
	 ruta_y=0.3*uniresuly+robot_y;
	
	       
        //COntrol diferencial
	 float alfa=0.5;
	 float beta=0.5;
 	 float vmax=0.5;
         float wmax=0.5;
	 float ex=ruta_x-robot_x;
	 float ey=ruta_y-robot_y;
	 float etheta=atan2(ey,ex)-robot_a;
	 float v;
	 float w;
	//COndicion para que ethtea tenga valores siempre
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
	   
    
	pub_cmd_vel.publish(msg_cmd_vel);
	pub_markers.publish(get_attraction_arrow());
	pub_markers.publish(get_rejection_arrow());
	pub_markers.publish(get_resulting_arrow());
	ros::spinOnce();
	loop.sleep();
    }
    return 0;
}
