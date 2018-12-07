#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <hardware_tools/DynamixelManager.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

bool newGoalPose=false;

float goalPos_simul[7]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0};
float goalSpeeds_simul[7]={0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
float positions[7] = {0, 0, 0, 0, 0, 0, 0};
int zero_arm[7] = {1543, 1694, 1742, 2100, 2048, 2066, 1050};

void callbackArmGoalPose(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
  std::cout << "left_arm_node.-> Reciving new goal left arm pose." << std::endl;
  for(int i =0; i<7; i++){
    goalPos_simul[i] = msg->data[i];
    goalSpeeds_simul[i] = 0.1;
  }

  if(msg->data.size() == 14){
    for(int i = 0; i<7;i++)
      goalSpeeds_simul[i] = msg->data[i+7];
  }
   
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "left_arm_node");
  ros::NodeHandle n;

  ros::Subscriber subGoalPos = n.subscribe("/hardware/left_arm/goal_pose", 1, callbackArmGoalPose);
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
  ros::Publisher pubArmPose = n.advertise<std_msgs::Float32MultiArray>("left_\
arm/current_pose", 1);
  
  ros::Rate rate(20);
  
  std::string names[7] = {"la_1_joint", "la_2_joint", "la_3_joint", "la_4_joint", "la_5_joint", "la_6_joint", "la_7_joint"};
   

  sensor_msgs::JointState jointStates;
  jointStates.name.insert(jointStates.name.begin(), names, names + 7);
  jointStates.position.insert(jointStates.position.begin(), positions, positions + 7);
  
  std_msgs::Float32MultiArray msgCurrPose;
  msgCurrPose.data.resize(7);
  
  while(ros::ok())
    {
      /*      float CT1=px-ax(l5+l6);
      float ST1=py+ay(l5+l6);
      float T1=atan(ST1,CT1);*/
      // for(int i=0; i<20; i++)
      // {
      
      jointStates.position[0]=5;
      jointStates.position[1]=1;
           for(int i = 0; i < 7; i++)
	 msgCurrPose.data[i] = jointStates.position[i];
       
       // }
      jointStates.header.stamp = ros::Time::now();
      joint_pub.publish(jointStates);
      pubArmPose.publish(msgCurrPose);
      ros::spinOnce();
      rate.sleep();
    }
  return 0;
}
  

