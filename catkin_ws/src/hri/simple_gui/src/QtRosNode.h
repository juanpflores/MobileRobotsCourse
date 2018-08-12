#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "geometry_msgs/Twist.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    ros::NodeHandle* n;
    ros::Publisher  pubCmdVel;
    geometry_msgs::Twist cmd_vel;
    bool publishing_cmd_vel;
    bool gui_closed;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void publish_cmd_vel(float linear_frontal, float linear_lateral, float angular);
    void start_publishing_cmd_vel(float linear_frontal, float linear_lateral, float angular);
    void stop_publishing_cmd_vel();
    
signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
