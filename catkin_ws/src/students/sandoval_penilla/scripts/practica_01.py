#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Practica01(object):
    mov = False;
    obs = False;
    
    
    
    def __init__(self):
        #creamos un nodo
        print("Creando Nodo")
        rospy.init_node("practica01_node",anonymous=False)
        #Publisher
        self.pub = rospy.Publisher('/hardware/mobile_base/cmd_vel',
                              Twist,
                              queue_size=1)
        #Subscriber
        rospy.Subscriber('/std_msgs/bool',
                         Bool,
                         self.callback_move)
                         #Subscriber
        rospy.Subscriber('/hardware/scan',
                         LaserScan,
                         self.callback_scan)
                         
    def callback_move(self,msg):
        self.mov = msg.data
        print("Recibed: {}".format(self.mov))
        
    def callback_scan(self,msg):
        front_laser = msg.ranges[len(msg.ranges)/2]
        self.obs = front_laser < 0.5        
        if self.obs:
            #self.mov = False
            print("--------")
            print("Robot Blocked?: {}".format(self.mov))
            print("Obstacle?: {}".format(self.obs))
            print("Alert!!! Distance: {:9.2f}".format(front_laser))
        else:
            #self.mov = True
            print("--------")
            print("Robot Blocked?: {}".format(self.mov))
            print("Obstacle?: {}".format(self.obs))
            print("Distance: {:9.2f}".format(front_laser))
            
    
        
    def main(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            msg = Twist()
            msg.linear.x = 0.4
            if self.obs or self.mov:
                msg.linear.x = 0
            self.pub.publish(msg)
            rate.sleep()
            
if __name__ == "__main__":
    try:
        prac = Practica01()
        prac.main()
        
        
    except rospy.ROSInterruptException:
        pass
