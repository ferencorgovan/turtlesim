#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class vel_manipulator:
 
    def __init__(self):
        pub_topic_name ="/turtle1/cmd_vel"
        sub_topic_name ="/turtle1/pose"
        
        self.pub = rospy.Publisher(pub_topic_name, Twist, queue_size=10)
        self.number_subcriber = rospy.Subscriber(sub_topic_name, Pose, self.pose_callback)
        self.velocity_msg = Twist()

    def pose_callback(self, msg):
        while True:
            self.velocity_msg.linear.x = 0.5
            self.velocity_msg.angular.z = 0.5
            break
        

        self.pub.publish(self.velocity_msg)

if __name__ == '__main__':
    node_name ="TurtlesimSaver"
    rospy.init_node(node_name)
    vel_manipulator()
    rospy.spin()       