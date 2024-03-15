#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np

class VideoOutputNode:
    def __init__(self):
        rospy.init_node('video_output')
        rospy.loginfo('VideoOutput node initialized')

        # Subscriber for detection messages
        self.detection_sub = rospy.Subscriber('detections', Detection2DArray, self.detection_callback)

        self.bridge = CvBridge()
        self.window_name = 'Video Output'

    def detection_callback(self, msg):
        if len(msg.detections) == 0:
            rospy.logwarn('No detections received.')
            return

        # Get the first detection
        detection = msg.detections[0]

        # Extract bounding box information
        bbox = detection.bbox
        width = int(bbox.size_x)
        height = int(bbox.size_y)

        # Convert float center coordinates to integers
        center_x = int(bbox.center.x)
        center_y = int(bbox.center.y)

        # Draw bounding box on the image
        cv_image = np.zeros((height, width, 3), dtype=np.uint8)  # Create black image
        cv2.rectangle(cv_image, (0, 0), (width, height), (255, 255, 255), 2)  # Draw white rectangle

        # Display the image
        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(1)

        # Log detection information
        rospy.loginfo(f'Received detection message - Object ID: {detection.results[0].id}, Score: {detection.results[0].score}')


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = VideoOutputNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
