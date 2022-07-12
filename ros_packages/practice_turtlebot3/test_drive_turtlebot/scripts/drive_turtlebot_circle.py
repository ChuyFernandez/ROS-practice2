#!/usr/bin/env python

# Crearemos un nodo que manejara el turtlebot en circulos

import rospy
from geometry_msgs.msg import Twist

if __name__=='__main__':

    node_name="drive_turtlebot_circle"
    topic_name="/cmd_vel"

    rospy.init_node(node_name)
    pub=rospy.Publisher(topic_name, Twist, queue_size=1)
    rate=rospy.Rate(2)

    move=Twist()
    move.linear.x=0.2   # Es el eje para el movimiento lineal del robot
    move.angular.z=0.5  # Es el eje de rotacion del robot       

    rospy.loginfo('Listo, se empezara a publicar en el tema: %s'%(topic_name))

    try:
        while not rospy.is_shutdown():
            pub.publish(move)
            rate.sleep()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo('Se ha apagado el nodo')
        