#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Float64MultiArray, String  #tipo de mensagem que sera enviado
from cv_bridge import CvBridge
import cv2
import gc
import numpy as np
import math
import jetson.inference
import jetson.utils    

class DetectNetNode:
    def __init__(self):
        rospy.init_node('detectnet')
        rospy.loginfo('O node Detecnet foi iniciado!')

        #https://github.com/dusty-nv/jetson-inference/blob/master/docs/detectnet-console-2.md
        self.net = jetson.inference.detectNet("trafficcamnet", threshold=0.5)#trafficcamnet #ssd-mobilenet-v2
        
        self.detection_pub = rospy.Publisher('detections', Detection2DArray, queue_size=25)
        self.image_pub = rospy.Publisher('detections_with_boxes', Image, queue_size=25)

        ##Recebe uma imagem
        self.image = Image()      
        self.depth = Image()      
        ##Instancia um objeto do tipo bridge para fazer a conversão de opencv para image               
        self.bridge = CvBridge()   
        self.bridgeDepth = CvBridge()     
        ##Instancia um objeto do tipo float que servira para enviar as coordenadas dos objetos       
        self.objectsDetectnet = String()
        self.imageDetectnet = Image()  

        #recebimento de mensagens
        self.subImage = rospy.Subscriber('TPC1Camera', Image, self.callbackImage)
        self.subDepth = rospy.Subscriber('TPC2Depth', Image, self.callbackDepth)

        #publicação 
        self.pubImgDetectnet = rospy.Publisher('ImgDetectnet', Image, queue_size=1)           #guarda os parametros para o envio da imagem da Detectnet
        self.pubObjectDetectnet = rospy.Publisher('ObjectDetectnet', String, queue_size=1)         #guarda os parametros para o envio das informações da Detectnet
    
    #callback da profundidade
    def callbackDepth(self, msg_depth):
        self.depth = self.bridgeDepth.imgmsg_to_cv2(msg_depth,'32FC1')         
    
    def getObjetcs(self):
        #numpy array -> cudaImage
        cuda_image = jetson.utils.cudaFromNumpy(self.image)

        objectsDetectnet = {}
        objectDetectnet = {
            "trackID": 0,
            "classId": 0,
            "class": "",
            "coords": [0, 0, 0, 0],
            "conf": 0,
            "distance": 0
        }
        
        detections = self.net.Detect(cuda_image, overlay="none")
        objectDetectnetID = 0
        for detection in detections:
            objectDetectnet["trackID"] = detection.TrackID
            objectDetectnet["classId"] = detection.ClassID
            objectDetectnet["class"]   = (self.net.GetClassLabel(detection.ClassID))
            objectDetectnet["coords"]  = [int(i) for i in detection.ROI] 
            objectDetectnet["center"]  = [int(i) for i in detection.Center]
            objectDetectnet["conf"]    = detection.Confidence
            objectDetectnet["distance"] = round(self.getDistance(objectDetectnet["coords"]), 2)
            
            objectDetectnetID = objectDetectnetID + 1
            objectsDetectnet[str(objectDetectnetID)] = objectDetectnet

            self.getImageDetectnet(objectDetectnet)
            
        return str(objectsDetectnet)

    def getImageDetectnet(self, objectDetectnet): 
        object_class = objectDetectnet["class"] 
        bbox = objectDetectnet["coords"]
        object_distance = objectDetectnet["distance"]
        x, y = objectDetectnet["center"]

        cv2.putText(self.imageDetectnet, object_class, (bbox[0]+5, bbox[1]-5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.6, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.rectangle(self.imageDetectnet, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
        cv2.putText(self.imageDetectnet, "dist: "+str(object_distance), (bbox[0]+5, bbox[1]+25), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.6, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.circle(self.imageDetectnet, [x, y], 10, (255, 0, 0), 5)

    def getDistance(self, coords):
        distanciaMin = np.Inf
        x1, y1, x2, y2 = coords
        recorte = self.depth[y1:y2, x1:x2].copy() #Faz um recorte da image, analisa apenas uma parte da imagem (retangulo)
        distanciaMin = np.nanmin(recorte)         #Pega a menor idstancia sem ser um nan    
        if(distanciaMin < 0):
            distanciaMin = 0
    
        return distanciaMin
        
    #callback para a imagem
    def callbackImage(self, image):
        self.image = self.bridge.imgmsg_to_cv2(image,'bgra8')
        self.imageDetectnet = self.image
        if(self.depth is not None):
            self.objectsDetectnet.data = self.getObjetcs()
            print(self.objectsDetectnet.data )
            self.pubObjectDetectnet.publish(self.objectsDetectnet)
            self.pubImgDetectnet.publish(self.bridge.cv2_to_imgmsg(self.imageDetectnet,"bgra8"))
        
if __name__ == '__main__':
    try:
        node = DetectNetNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass