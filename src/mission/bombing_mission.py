#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
from tracker_algorithm import TrackerAlgorithm
from darknet_ros_msgs.msg import BoundingBoxes
import sys
import time
from datetime import datetime

class_name = 'person'
probability_threshold = 0.1

class TargetTracker:
    def __init__(self, class_name, probability_threshold):
        rospy.init_node('target_tracker', anonymous=True)

        # Subscribe to the bounding boxes and image topics
        self.bbox_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbox_callback)
        self.image_sub = rospy.Subscriber('/darknet_ros/detection_image', Image, self.image_callback)

        # Publisher for the tracked position
        self.position_pub = rospy.Publisher('/target_position', Point, queue_size=10)
        
        # New publisher for drop status
        self.drop_pub = rospy.Publisher('/drop', Int32, queue_size=10)

        # Initialize TrackerAlgorithm with custom parameters
        self.tracker = TrackerAlgorithm()
        self.tracker.set_parameters(class_name, probability_threshold)

        # Initialize timer and validation flag
        self.target_locked_time = None
        self.target_validated = False
        self.target_lost_time = None  # To track when the target was lost

        # Initialize drop status variable
        self.drop_status = 0

    def bbox_callback(self, data):
        position = self.tracker.bbox_callback(data)
        
        if self.tracker.target_locked:
            if self.target_locked_time is None:
                # Start the timer when the target is first locked
                self.target_locked_time = time.time()
            elif not self.target_validated and (time.time() - self.target_locked_time) >= 5:
                # Validate the target after 5 seconds
                self.target_validated = self.tracker.is_target_stable()
                
            if self.target_validated and self.tracker.target_centroid:
                # Publish the target's centroid when validated
                self.position_pub.publish(Point(self.tracker.target_centroid[0], self.tracker.target_centroid[1], 0))
            else:
                # Publish (0, 0) if not validated
                self.position_pub.publish(Point(0, 0, 0))
        else:
            # Reset the target state if it is lost
            self.position_pub.publish(Point(0, 0, 0))
            self.reset_tracking()

    def reset_tracking(self):
        # Reset timer and validation flag
        self.target_locked_time = None
        self.target_validated = False
        self.target_lost_time = None

    def image_callback(self, msg):
        self.current_frame = imgmsg_to_cv2(msg)
        self.current_frame = np.copy(self.current_frame)

        if self.tracker.target_locked and self.tracker.bounding_box:
            # Set bounding box color based on validation status
            if self.drop_status == 1:
                box_color = (0, 255, 0)  # Green if dropped
            else:
                box_color = (0, 255, 255) if self.target_validated else (0, 0, 255)

            # Get the dimensions of the image
            height, width, _ = self.current_frame.shape

            # Calculate the center of the image
            center_x, center_y = width // 2, height // 2

            # Draw the larger circle hole (4x the size of the mini circle)
            radius_large = 60  # Radius of the larger circle hole
            thickness_large = 2  # Thickness of the larger circle hole
            circle_color_large = (255, 255, 255)  # White circle outline
            cv2.circle(self.current_frame, (center_x, center_y), radius_large, circle_color_large, thickness_large)

            # Draw the smaller circle (mini circle) at the center of the frame
            radius_small = 5  # Radius of the smaller circle (mini circle)
            thickness_small = 2  # Thickness of the smaller circle
            circle_color_small = (0, 255, 0)  # Green circle outline
            cv2.circle(self.current_frame, (center_x, center_y), radius_small, circle_color_small, thickness_small)

            # Check if the target centroid is within the smaller circle
            if self.tracker.target_centroid:
                target_x, target_y = self.tracker.target_centroid
                dot_color = (0, 0, 255)  # Red color for the dot
                dot_radius = 5
                cv2.circle(self.current_frame, (int(target_x), int(target_y)), dot_radius, dot_color, -1)

                # Check if the target centroid is within the smaller circle
                distance = np.sqrt((target_x - center_x) ** 2 + (target_y - center_y) ** 2)
                if distance <= radius_small and self.target_validated:
                    box_color = (0, 255, 0)  # Green color for the rectangle if the target is within the circle
                    self.drop_status = 1  # Set drop status to 1 if within the circle
                else:
                    self.drop_status = 0  # Set drop status to 0 otherwise
            else:
                self.drop_status = 0  # Set drop status to 0 if no target

            # Publish drop status
            self.drop_pub.publish(Int32(self.drop_status))

            # Create an overlay image with the same size as the original frame
            overlay = self.current_frame.copy()

            if self.tracker.bounding_box:
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
                if self.tracker.bounding_box:
                    cv2.putText(
                        self.current_frame,
                        self.tracker.class_label,
                        (self.tracker.bounding_box[0], self.tracker.bounding_box[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,  # Smaller font size
                        box_color, 1  # Smaller thickness
                    )

        else:
            # Clear the frame if no valid target is detected
            self.current_frame = np.copy(self.current_frame)

        # Get the current date and time
        now = datetime.now()
        date_time_str = now.strftime("%Y-%m-%d %H:%M:%S")
        
        # Add text to the frame
        text_drop_status = "Drop Status: {}".format('Dropped' if self.drop_status == 1 else 'Not Dropped')
        text_date_time = "Date Time: {}".format(date_time_str)
        text_delivery_mission = "Bombing Mission"  # Changed text

        # Define a local height variable to avoid referencing the global height
        local_height = self.current_frame.shape[0]

        # Put Drop Status and Date Time on the top left corner
        cv2.putText(self.current_frame, text_drop_status, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(self.current_frame, text_date_time, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

        # Put Bombing Mission on the bottom left corner using the local height variable
        cv2.putText(self.current_frame, text_delivery_mission, (10, local_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

        # Display the frame with the bounding box, dot, and label
        cv2.imshow('Target Tracking', self.current_frame)
        cv2.waitKey(1)

def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8")
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(
        shape=(img_msg.height, img_msg.width, 3),
        dtype=dtype, buffer=img_msg.data
    )
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return np.copy(image_opencv)

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data) // img_msg.height
    return img_msg

if __name__ == '__main__':
    try:
        tracker = TargetTracker(class_name, probability_threshold)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
