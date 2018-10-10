#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Practica01:
    """
    Empleando el paquete de surge_et_embula, se le publicara una velocidad lineal al robot,
    utilizando el sensor laser incorporado el robot debe detenerse si detecta un obstaculo
    a una distancia de 0.5 metros. Tambien se puede bloquear el movimiento del mismo
    haciendo uso de un rostopic pub 
    """
    mov = False
    obs = False
    front_laser = 0

    def __init__(self):
        #creamos un nodo con el nombre de practica01_node
        print("Creando Nodo")
        rospy.init_node("practica01_node",anonymous=False)
        #Publisher que publica al topico /hardware/mobile_base/cmd_vel' 
        #el tipo del mensaje es de tipo Twist
        self.pub = rospy.Publisher('/hardware/mobile_base/cmd_vel',
                              Twist,
                              queue_size=1)
        #Subscriber, escuchamos al topico /std_msgs/bool al cual se le'
        #publicara mediante un rostopic pub, esto nos permitira bloquer al robot  
        rospy.Subscriber('/std_msgs/bool',
                         Bool,
                         self.callback_move)
        #Subscriber, escuchamos publisher /hardware/scan el cual nos
        #proporciona un arreglo con la distancia en metros del laser 
        rospy.Subscriber('/hardware/scan',
                         LaserScan,
                         self.callback_scan)

    #Las funciones callback_move y callback_scan se ejecutan cada vez
    #que reciben un mesaje
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
        #La funcion se ejecutara 15 en un segundo
        rate = rospy.Rate(15)
        #Se ejecutara hasta que el nodo practica01_node sea eliminado
        while not rospy.is_shutdown():
            #creamos un mensaje de tipo Twist
            msg = Twist()
            #Modificamos el mensaje
            msg.linear.x = 0.4
            #Si el robot detecta un obstaculo o el robot esta bloqueado
            #se mandara una velocidad de cero al robot
            if self.obs or self.mov:
                msg.linear.x = 0
            #Publicamos el mensaje
            self.pub.publish(msg)
            #El ciclo while se dormira el tiempo necesario para que se cumpla
            #el rate de 15 segundos
            rate.sleep()

if __name__ == "__main__":
    try:
        prac = Practica01()
        prac.main()
    except rospy.ROSInterruptException:
        pass
