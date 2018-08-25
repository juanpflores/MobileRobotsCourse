#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Practica01(object):

    mov = False
    obs = False
    front_laser = 0

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
        self.front_laser = msg.ranges[len(msg.ranges)/2]
        self.obs = self.front_laser < 0.5
        print(self)

    def __str__(self):
        return  """---------------
Robot Blocked?: {}
Obstacle?:      {}
Distance:       {:.2f}""".format(self.mov,self.obs,self.front_laser)


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
