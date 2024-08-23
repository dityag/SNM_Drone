#!/usr/bin/env python

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

# Global state variable
current_state = State()

def state_callback(msg):
    global current_state
    current_state = msg

def send_velocity(velocity):
    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    rospy.sleep(1)  # Allow publisher to establish
    rate = rospy.Rate(10)  # 10 Hz
    twist_stamped = TwistStamped()
    twist_stamped.header = Header(stamp=rospy.Time.now())
    twist_stamped.twist.linear.x = velocity[0]  # x linear velocity
    twist_stamped.twist.linear.y = velocity[1]  # y linear velocity
    twist_stamped.twist.linear.z = velocity[2]  # z linear velocity
    twist_stamped.twist.angular.z = velocity[3]  # z angular velocity (yaw rate)

    # Continuously publish velocity setpoints
    while not rospy.is_shutdown():
        twist_stamped.header.stamp = rospy.Time.now()
        pub.publish(twist_stamped)
        rospy.loginfo("Publishing velocity: x=%.2f, y=%.2f, z=%.2f, yaw_rate=%.2f",
                      velocity[0], velocity[1], velocity[2], velocity[3])
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node('setpoint_velocity_node')
    
    # Subscribe to the state topic
    rospy.Subscriber('/mavros/state', State, state_callback)

    rospy.sleep(5)  # Wait for the subscriber to receive the state

    # Define the velocity (x, y, z, yaw_rate)
    velocity = [0.3, 0.0, 0.0, 0.0]  # Move forward with 1 m/s

    # Send velocity setpoint
    send_velocity(velocity)
