#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
from tracker_algorithm import TrackerAlgorithm
from darknet_ros_msgs.msg import BoundingBoxes
import time
import sys
from datetime import datetime

class_name = 'person'
probability_threshold = 0.1

class TargetTracker:
    def __init__(self, class_name, probability_threshold):
        rospy.init_node('target_tracker', anonymous=True)

        # Initialize TrackerAlgorithm with custom parameters
        self.tracker = TrackerAlgorithm()
        self.tracker.set_parameters(class_name, probability_threshold)
        
        if self.tracker is None:
            rospy.logerr('Failed to initialize TrackerAlgorithm')
        else:
            rospy.loginfo('TrackerAlgorithm initialized successfully')

        # Subscribe to the bounding boxes and image topics
        self.bbox_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbox_callback)
        self.image_sub = rospy.Subscriber('/darknet_ros/detection_image', Image, self.image_callback)

        # Publishers for the target position, drop status, fire position, and fire status
        self.target_position_pub = rospy.Publisher('/target_position', Point, queue_size=10)
        self.drop_pub = rospy.Publisher('/drop', Int32, queue_size=10)
        self.fire_position_pub = rospy.Publisher('/fire_position', Point, queue_size=10)
        self.fire_pub = rospy.Publisher('/fire', Int32, queue_size=10)  # Added fire publisher

        # Timer and flag for target validation
        self.target_locked_time = None
        self.target_validated = False
        self.drop_status = 0  # Initialize drop status

    def bbox_callback(self, data):
        if not hasattr(self, 'tracker'):
            rospy.logerr('Tracker attribute is missing')
            return
        
        position = self.tracker.bbox_callback(data)
        if position:
            self.target_locked_time = time.time()  # Reset the timer when the target is detected
        else:
            # No need to publish position here anymore
            pass

        if self.tracker.target_locked:
            if self.target_locked_time is None:
                # Start the timer when the target is first locked
                self.target_locked_time = time.time()
            elif not self.target_validated and (time.time() - self.target_locked_time) >= 5:
                # Validate the target after 1 second
                self.target_validated = self.tracker.is_target_stable()

    def image_callback(self, msg):
        if not hasattr(self, 'tracker'):
            rospy.logerr('Tracker attribute is missing')
            return

        self.current_frame = imgmsg_to_cv2(msg)
        self.current_frame = np.copy(self.current_frame)

        # Get the center of the frame
        frame_center = (self.current_frame.shape[1] // 2, self.current_frame.shape[0] // 2)

        # Draw a circle hole at the center of the frame
        circle_color = (0, 0, 255)  # Red
        circle_radius = 10  # Radius of the circle
        cv2.circle(self.current_frame, frame_center, circle_radius, circle_color, 2)  # Circle hole
        
        cv2.circle(self.current_frame, frame_center, 110, (255, 255, 255), 2)  # Circle hole

        if self.tracker.target_locked:
            if self.tracker.target_centroid is not None and self.tracker.bounding_box is not None:
                # Determine the color of the bounding box based on validation
                box_color = (0, 255, 0) if self.target_validated else (0, 0, 255)
                
                # Draw a green dot at the target's centroid
                cv2.circle(self.current_frame, self.tracker.target_centroid, 5, box_color, -1)

                # Check if the target is centered
                distance_to_center = np.linalg.norm(np.array(self.tracker.target_centroid) - np.array(frame_center))
                central_region_radius = 20  # Radius to consider target as centered

                print(f"Distance to center: {distance_to_center}, Central region radius: {central_region_radius}")  # Debug

                if distance_to_center <= central_region_radius and self.target_validated:
                    # Target is centered and validated; change the circle color to green and set drop_status to 1
                    circle_color = (0, 255, 0)  # Green
                    self.drop_status = 1
                else:
                    # Target is not centered or not validated; keep the circle color red and set drop_status to 0
                    circle_color = (0, 0, 255)  # Red
                    self.drop_status = 0

                print(f"Drop status: {self.drop_status}")  # Debug

                # Redraw the circle with the updated color
                cv2.circle(self.current_frame, frame_center, circle_radius, circle_color, 2)  # Circle hole

                # Create an overlay image with the same size as the original frame
                overlay = self.current_frame.copy()
                
                # Draw a semi-transparent rectangle on the overlay image
                alpha = 0.5  # Transparency factor (0.0 to 1.0)
                cv2.rectangle(
                    overlay,
                    (self.tracker.bounding_box[0], self.tracker.bounding_box[1]),
                    (self.tracker.bounding_box[2], self.tracker.bounding_box[3]),
                    box_color,
                    thickness=cv2.FILLED
                )
                
                # Blend the overlay with the original image
                cv2.addWeighted(overlay, alpha, self.current_frame, 1 - alpha, 0, self.current_frame)
                
                # Put the class label text above the bounding box
                cv2.putText(
                    self.current_frame,
                    self.tracker.class_label,
                    (self.tracker.bounding_box[0], self.tracker.bounding_box[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,  # Smaller font scale
                    box_color, 2
                )
                
                # Increment stable frames counter if the target is still in the same position
                if self.tracker.is_target_stable():
                    self.tracker.stable_frames += 1
                    if self.tracker.stable_frames >= self.tracker.max_stable_frames:
                        self.tracker.reset_target()
                else:
                    self.tracker.stable_frames = 0
                
                # Publish the target position
                if self.tracker.target_centroid is not None:
                    target_position = Point()
                    target_position.x = self.tracker.target_centroid[0]
                    target_position.y = self.tracker.target_centroid[1]
                    target_position.z = 0  # Set z to 0, or any value as required
                    self.target_position_pub.publish(target_position)
            else:
                # Publish target position as (0, 0, 0) if the target is not detected
                target_position = Point()
                target_position.x = 0
                target_position.y = 0
                target_position.z = 0
                self.target_position_pub.publish(target_position)
                self.drop_status = 0
        else:
            # Publish target position as (0, 0, 0) if the target is not locked
            target_position = Point()
            target_position.x = 0
            target_position.y = 0
            target_position.z = 0
            self.target_position_pub.publish(target_position)
            self.drop_status = 0
            self.tracker.reset_target()

        # Publish the fire position as the center of the frame
        fire_position = Point()
        fire_position.x = frame_center[0]
        fire_position.y = frame_center[1]
        fire_position.z = 0  # Set z to 0, or any value as required
        self.fire_position_pub.publish(fire_position)

        # Publish drop status
        self.drop_pub.publish(self.drop_status)

        # Always publish 0 to the /fire topic
        self.fire_pub.publish(0)  # Added line to publish 0 to /fire

        # Add drop status and date-time to the top-left corner of the frame
        status_text = 'Dropping' if self.drop_status == 1 else 'Not Dropping'
        date_time_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        cv2.putText(self.current_frame, f'Drop Status: {status_text}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)  # Smaller text
        cv2.putText(self.current_frame, f'DateTime: {date_time_str}', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)  # Smaller text
        
        # Add "Dropping Mission" text to the bottom-left corner
        dropping_mission_text = 'Dropping Mission'
        text_size = 0.4
        text_color = (255, 255, 255)  # White
        text_thickness = 1
        text_position = (10, self.current_frame.shape[0] - 10)  # Bottom-left corner
        cv2.putText(self.current_frame, dropping_mission_text, text_position, cv2.FONT_HERSHEY_SIMPLEX, text_size, text_color, text_thickness)

        # Display the frame with the bounding box, green dot, label, and additional text
        cv2.imshow('Target Tracking', self.current_frame)
        cv2.waitKey(1)

def imgmsg_to_cv2(img_msg):
    try:
        return np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
    except Exception as e:
        rospy.logerr('Failed to convert Image message to OpenCV image: {}'.format(str(e)))
        return None

if __name__ == '__main__':
    try:
        target_tracker = TargetTracker(class_name, probability_threshold)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('TargetTracker node terminated.')
    finally:
        cv2.destroyAllWindows()
