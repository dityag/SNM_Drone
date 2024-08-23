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

        # Subscribe to the bounding boxes and image topics
        self.bbox_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbox_callback)
        self.image_sub = rospy.Subscriber('/darknet_ros/detection_image', Image, self.image_callback)

        # Publishers for the fire position and fire status
        self.fire_position_pub = rospy.Publisher('/fire_position', Point, queue_size=10)
        self.fire_pub = rospy.Publisher('/fire', Int32, queue_size=10)

        # Initialize TrackerAlgorithm with custom parameters
        self.tracker = TrackerAlgorithm()
        self.tracker.set_parameters(class_name, probability_threshold)

        # Timer and flag for target validation
        self.target_locked_time = None
        self.target_validated = False
        self.fire_status = 0  # Initialize fire status

    def bbox_callback(self, data):
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
            elif not self.target_validated and (time.time() - self.target_locked_time) >= 0.2:
                # Validate the target after 0.2 seconds
                self.target_validated = self.tracker.is_target_stable()

    def image_callback(self, msg):
        if not hasattr(self, 'tracker'):
            rospy.logerr('Tracker attribute is missing')
            return

        self.current_frame = imgmsg_to_cv2(msg)
        self.current_frame = np.copy(self.current_frame)

        # Get the center of the frame
        frame_center = (self.current_frame.shape[1] // 2, self.current_frame.shape[0] // 2)

        # Draw the + symbol at the center of the frame (default color red)
        plus_color = (0, 0, 255)  # Red
        cv2.line(self.current_frame, (frame_center[0] - 10, frame_center[1]), (frame_center[0] + 10, frame_center[1]), plus_color, 2)
        cv2.line(self.current_frame, (frame_center[0], frame_center[1] - 10), (frame_center[0], frame_center[1] + 10), plus_color, 2)

        if self.tracker.target_locked:
            if self.tracker.target_centroid is not None and self.tracker.bounding_box is not None:
                # Determine the color of the bounding box based on validation
                box_color = (0, 255, 0) if self.target_validated else (0, 0, 255)
                
                # Draw a green dot at the target's centroid
                cv2.circle(self.current_frame, self.tracker.target_centroid, 5, box_color, -1)

                # Check if the target is centered
                distance_to_center = np.linalg.norm(np.array(self.tracker.target_centroid) - np.array(frame_center))
                central_region_radius = 20  # Radius to consider target as centered

                if distance_to_center <= central_region_radius:
                    # Target is centered; change the + symbol to green
                    plus_color = (0, 255, 0)  # Green
                    self.fire_status = 1
                else:
                    # Target is not centered; keep the + symbol red
                    plus_color = (0, 0, 255)  # Red
                    self.fire_status = 0

                # Redraw the + symbol with the updated color
                cv2.line(self.current_frame, (frame_center[0] - 10, frame_center[1]), (frame_center[0] + 10, frame_center[1]), plus_color, 2)
                cv2.line(self.current_frame, (frame_center[0], frame_center[1] - 10), (frame_center[0], frame_center[1] + 10), plus_color, 2)

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
                
                # Publish the fire position
                if self.tracker.target_centroid is not None:
                    target_position = Point()
                    target_position.x = self.tracker.target_centroid[0]
                    target_position.y = self.tracker.target_centroid[1]
                    target_position.z = 0  # Set z to 0, or any value as required
                    self.fire_position_pub.publish(target_position)
            else:
                # Publish only fire status
                self.fire_status = 0
        else:
            # Publish only fire status
            self.fire_status = 0
            self.tracker.reset_target()

        # Publish fire status
        self.fire_pub.publish(self.fire_status)

        # Add fire status and date-time to the top-left corner of the frame
        status_text = 'Firing' if self.fire_status == 1 else 'Not Firing'
        date_time_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        cv2.putText(self.current_frame, f'Fire Status: {status_text}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)  # Smaller text
        cv2.putText(self.current_frame, f'DateTime: {date_time_str}', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)  # Smaller text
        
        # Add "Firing Mission" text to the bottom-left corner
        firing_mission_text = 'Firing Mission'
        text_size = 0.4
        text_color = (255, 255, 255)  # White
        text_thickness = 1
        text_position = (10, self.current_frame.shape[0] - 10)  # Bottom-left corner
        cv2.putText(self.current_frame, firing_mission_text, text_position, cv2.FONT_HERSHEY_SIMPLEX, text_size, text_color, text_thickness)

        # Display the frame with the bounding box, green dot, label, and additional text
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
        # Create an instance of TargetTracker with custom parameters if needed
        tracker = TargetTracker(class_name, probability_threshold)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
