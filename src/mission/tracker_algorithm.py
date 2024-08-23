import numpy as np
from geometry_msgs.msg import Point
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

class TrackerAlgorithm:
    def __init__(self):
        self.class_name = None
        self.probability_threshold = None
        self.target_centroid = None
        self.bounding_box = None
        self.class_label = None
        self.target_locked = False
        self.missed_frames = 0
        self.max_missed_frames = 10
        self.stable_frames = 0
        self.max_stable_frames = 30

    def set_parameters(self, class_name, probability_threshold):
        self.class_name = class_name
        self.probability_threshold = probability_threshold

    def bbox_callback(self, data):
        if self.target_locked:
            found = False
            for bbox in data.bounding_boxes:
                if self.is_same_target(bbox):
                    self.update_target(bbox)
                    found = True
                    self.missed_frames = 0
                    break

            if not found:
                self.missed_frames += 1
                if self.missed_frames >= self.max_missed_frames:
                    self.reset_target()
                else:
                    self.stable_frames = 0
        else:
            for bbox in data.bounding_boxes:
                if bbox.Class == self.class_name and bbox.probability >= self.probability_threshold:
                    return self.lock_target(bbox)  # Return the position for publishing

    def lock_target(self, bbox):
        self.target_centroid = self.calculate_centroid(bbox)
        self.bounding_box = self.get_bounding_box(bbox)
        self.class_label = bbox.Class
        self.target_locked = True
        self.stable_frames = 0
        # Return position with default z coordinate (0)
        return Point(x=self.target_centroid[0], y=self.target_centroid[1], z=0)

    def update_target(self, bbox):
        self.target_centroid = self.calculate_centroid(bbox)
        self.bounding_box = self.get_bounding_box(bbox)
        self.class_label = bbox.Class

    def is_same_target(self, bbox):
        if self.target_centroid is None or self.bounding_box is None:
            return False

        current_centroid = self.calculate_centroid(bbox)
        distance = np.linalg.norm(np.array(current_centroid) - np.array(self.target_centroid))
        bbox_similarity = self.calculate_bbox_similarity(bbox)
        return distance < 75 and bbox_similarity > 0.5

    def calculate_centroid(self, bbox):
        centroid_x = int((bbox.xmin + bbox.xmax) / 2)
        centroid_y = int((bbox.ymin + bbox.ymax) / 2)
        return (centroid_x, centroid_y)

    def calculate_bbox_similarity(self, bbox):
        if self.bounding_box is None:
            return 0

        new_bbox = self.get_bounding_box(bbox)
        old_bbox = self.bounding_box

        intersection_xmin = max(new_bbox[0], old_bbox[0])
        intersection_ymin = max(new_bbox[1], old_bbox[1])
        intersection_xmax = min(new_bbox[2], old_bbox[2])
        intersection_ymax = min(new_bbox[3], old_bbox[3])

        intersection_area = max(0, intersection_xmax - intersection_xmin) * max(0, intersection_ymax - intersection_ymin)
        old_bbox_area = (old_bbox[2] - old_bbox[0]) * (old_bbox[3] - old_bbox[1])
        new_bbox_area = (new_bbox[2] - new_bbox[0]) * (new_bbox[3] - new_bbox[1])

        union_area = old_bbox_area + new_bbox_area - intersection_area
        return intersection_area / union_area

    def get_bounding_box(self, bbox):
        return (int(bbox.xmin), int(bbox.ymin), int(bbox.xmax), int(bbox.ymax))

    def reset_target(self):
        self.target_centroid = None
        self.bounding_box = None
        self.class_label = None
        self.target_locked = False

    def is_target_stable(self):
        if self.target_centroid is None or self.bounding_box is None:
            return False

        centroid = self.target_centroid
        bbox = self.bounding_box
        return (centroid == self.calculate_centroid_from_bbox(self.get_bounding_box_from_current_frame()))

    def calculate_centroid_from_bbox(self, bbox):
        return self.calculate_centroid(bbox)

    def get_bounding_box_from_current_frame(self):
        if self.bounding_box is None:
            return None
        return BoundingBox(
            xmin=self.bounding_box[0],
            ymin=self.bounding_box[1],
            xmax=self.bounding_box[2],
            ymax=self.bounding_box[3],
            Class=self.class_label
        )
