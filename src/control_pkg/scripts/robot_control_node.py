#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from math import sin, cos

class RobotControlNode:
    def __init__(self):
        rospy.init_node('robot_control_node', anonymous=True)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Example: assuming two wheels with separate joint controllers
        self.left_wheel_pub = rospy.Publisher('/left_wheel_controller/command', Float64, queue_size=10)
        self.right_wheel_pub = rospy.Publisher('/right_wheel_controller/command', Float64, queue_size=10)

    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Example: convert linear and angular velocities to wheel velocities for a differential drive robot
        left_wheel_velocity = linear_velocity - 0.5 * angular_velocity
        right_wheel_velocity = linear_velocity + 0.5 * angular_velocity

        # Publish wheel velocities
        self.left_wheel_pub.publish(left_wheel_velocity)
        self.right_wheel_pub.publish(right_wheel_velocity)

def main():
    try:
        control_node = RobotControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

