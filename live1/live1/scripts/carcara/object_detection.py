#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
import jetson.inference
import jetson.utils

class DetectNetNode:
    def __init__(self):
        rospy.init_node('detectnet')
        rospy.loginfo('DetectNet node initialized')

        # Load detectNet model
        self.net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)

        # Publisher for detections
        self.detection_pub = rospy.Publisher('detections', Detection2DArray, queue_size=25)

        # Subscriber for image
        self.image_sub = rospy.Subscriber('TPC1Camera', Image, self.image_callback)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgra8")
        cuda_image = jetson.utils.cudaFromNumpy(cv_image)
        
        # Perform object detection
        detections = self.net.Detect(cuda_image, overlay="none")

        # Create Detection2DArray message
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        for detection in detections:
            det_msg = Detection2D()
            det_msg.header = msg.header
                    
            bbox = detection.ROI
            width = bbox[2] - bbox[0]  # Calcula a largura
            height = bbox[3] - bbox[1]  # Calcula a altura

            det_msg.bbox.size_x = width
            det_msg.bbox.size_y = height

            # Define o centro do retângulo delimitador
            det_msg.bbox.center.x = (bbox[0] + bbox[2]) / 2
            det_msg.bbox.center.y = (bbox[1] + bbox[3]) / 2
            det_msg.bbox.center.theta = 0.0

            hyp = ObjectHypothesisWithPose()
            hyp.id = detection.ClassID
            hyp.score = detection.Confidence

            det_msg.results.append(hyp)

            detections_msg.detections.append(det_msg)


        self.detection_pub.publish(detections_msg)

if __name__ == '__main__':
    try:
        node = DetectNetNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
