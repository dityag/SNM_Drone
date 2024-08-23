#!/usr/bin/env python

import rospy
from mavros_msgs.msg import State, GlobalPositionTarget
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import math

# Global state variable
current_state = State()
initial_position = None
current_position = None

def state_callback(msg):
    global current_state
    current_state = msg

def position_callback(msg):
    global current_position
    current_position = msg

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

def calculate_distance(pos1, pos2):
    # Calculate distance in meters using Haversine formula
    R = 6371000  # Radius of Earth in meters
    lat1, lon1 = pos1.latitude, pos1.longitude
    lat2, lon2 = pos2.latitude, pos2.longitude

    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

if __name__ == "__main__":
    rospy.init_node('setpoint_velocity_node')
    
    # Subscribe to the state and position topics
    rospy.Subscriber('/mavros/state', State, state_callback)
    rospy.Subscriber('/mavros/global_position/global', GlobalPositionTarget, position_callback)

    rospy.sleep(5)  # Wait for the subscribers to receive the state and position

    # Check if initial_position has been set
    rospy.loginfo("Waiting for initial position...")
    timeout = rospy.Time.now() + rospy.Duration(30)  # 30 seconds timeout
    while not rospy.is_shutdown() and initial_position is None and rospy.Time.now() < timeout:
        if current_position:
            initial_position = current_position
            rospy.loginfo("Initial position received.")
        else:
            rospy.loginfo("Waiting for position data...")
            rospy.sleep(1)

    if initial_position:
        # Define the velocity (x, y, z, yaw_rate)
        velocity = [0.3, 0.0, 0.0, 0.0]  # Move forward with 0.3 m/s
        
        # Start moving forward
        send_velocity(velocity)
        
        # Wait and check distance traveled
        rate = rospy.Rate(1)  # Check every second
        distance_traveled = 0
        while not rospy.is_shutdown() and distance_traveled < 10:
            if current_position:
                distance_traveled = calculate_distance(initial_position, current_position)
                rospy.loginfo("Distance traveled: %.2f meters", distance_traveled)
            rate.sleep()
        
        # Stop movement
        send_velocity([0.0, 0.0, 0.0, 0.0])
    else:
        rospy.logerr("Initial position not received. Cannot start movement.")
