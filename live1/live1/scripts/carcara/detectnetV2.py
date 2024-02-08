#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
import jetson.inference
import jetson.utils

flagDepthReceived = False

class DetectNetNode:
    def __init__(self):
        rospy.init_node('detectnet')
        rospy.loginfo('DetectNet node initialized')

        self.depth = Image() 
        self.bridgeDepth = CvBridge() 
        self.bridge = CvBridge()

        self.net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)

        self.detection_pub = rospy.Publisher('detections', Detection2DArray, queue_size=25)
        self.image_pub = rospy.Publisher('detections_with_boxes', Image, queue_size=25)

        self.image_sub = rospy.Subscriber('TPC1Camera', Image, self.image_callback)
        self.subImage = rospy.Subscriber('TPC2Depth', Image, self.callbackDepth)

        #callback da profundidade
    def callbackDepth(self, msg_depth):
        global flagDepthReceived
        flagDepthReceived = True
        self.depth = self.bridgeDepth.imgmsg_to_cv2(msg_depth,'32FC1')  

    def getDistance(self, coords):
        distancia = 0
        if(flagDepthReceived):
            x, y = coords
            depth_array = np.array(self.depth)
            distancia = depth_array[int(y), int(x)]
            if(distancia < 0):
                distancia = 0
 
        return distancia
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgra8")

        #numpy array -> cudaImage
        cuda_image = jetson.utils.cudaFromNumpy(cv_image)

        detections = self.net.Detect(cuda_image, overlay="none")

        for detection in detections:
            bbox = detection.ROI
            object_class = (self.net.GetClassLabel(detection.ClassID))
            cv2.putText(cv_image, object_class, (int(bbox[0])+5, int(bbox[1])-5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.6, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.rectangle(cv_image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (0, 255, 0), 2)
            object_distance = round(self.getDistance(detection.Center), 2)
            cv2.putText(cv_image, "dist: "+str(object_distance), (int(bbox[0])+5, int(bbox[1])+25), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.6, (0, 255, 0), 2, cv2.LINE_AA)

        #image back -> ROS format
        img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgra8")

        self.image_pub.publish(img_msg)

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        for detection in detections:
            det_msg = Detection2D()
            det_msg.header = msg.header
                    
            bbox = detection.ROI
            width = bbox[2] - bbox[0] 
            height = bbox[3] - bbox[1] 

            det_msg.bbox.size_x = width
            det_msg.bbox.size_y = height

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
