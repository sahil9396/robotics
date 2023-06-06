#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class vel_manipulator:

    def __init__(self) :
        # As the object of this class is created, then instantly it creates or run pushlisher
        pub_topic_name ="/turtle1/cmd_vel" 
        sub_topic_name = "/turtle1/pose"
        # here you have to write the topicname , datatype , size

        self.pub = rospy.Publisher( pub_topic_name, Twist , queue_size=10)
        self.number_sub = rospy.Subscriber(sub_topic_name , Pose , self.pose_callback)
        self.velocity_msg = Twist()

    def pose_callback(self , msg):
        print(msg.x)
        if (msg.x > 11):
            self.velocity_msg.linear.x = 0
            self.velocity_msg.angular.z = 0
        elif (msg.x < 1):
            self.velocity_msg.linear.x = 0
            self.velocity_msg.angular.z = 0
        else:
            # this shows the thickness of the sphere
            self.velocity_msg.linear.x = self.velocity_msg.linear.x  + 0.01

            # this is not rotating, but it is revoling around a object at 0 distance (if only it is given z axis)
            self.velocity_msg.angular.z = 3

        self.pub.publish(self.velocity_msg)



if __name__ == '__main__':
    name = "turtle_velocity"
    rospy.init_node(name)
    vel_manipulator()
    rospy.spin()
