#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2

class VideoOutputNode:
    def __init__(self):
        rospy.init_node('detectnet_output')
        rospy.loginfo('DetectnetOutput node initialized')

        self.image_sub = rospy.Subscriber('ImgDetectnet', Image, self.image_callback)

        self.bridge = CvBridge()
        self.window_name = 'Video Output'

    def image_callback(self, img_msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgra8")

        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(1)

    def run(self):
        print('Running')
        rospy.spin()

if __name__ == '__main__':
    try:
        node = VideoOutputNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
