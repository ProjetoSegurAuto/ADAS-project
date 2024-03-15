#!/usr/bin/env python3

##########
#Informações sobre esse Node:
#Nome: depth
#Descrição: pega a profundidade da imagem.

import math
import numpy as np
import ast
import cv2
import rospy
from sensor_msgs.msg import Image       #tipo de mensagem para enviar imagem
from std_msgs.msg import Float64, String        #tipo de mensagem usada para enviar a distancia
from cv_bridge import CvBridge, CvBridgeError       #converter open cv para imagem ros


class NodeDepth():

    def __init__(self):
        rospy.init_node('depth')
        rospy.loginfo('O node Depth foi iniciado!')
    
        self.bridge = CvBridge()             #Instancia um objeto do tipo bridge para fazer a conversão de opencv para msg_Image  
        
        self.msgImage = None 
        self.msgDepth = None              
        self.msgNave = None 
        self.msgObject = None

        self.objectDepth = Float64()  

        self.subImg = rospy.Subscriber('TPC1Camera', Image, self.callbackImagem)
        self.subNave = rospy.Subscriber('TPC8LKAnave', Image, self.callbackNave)
        self.subDepth = rospy.Subscriber('TPC2Depth',Image, self.callbackDepth)
        self.subObject = rospy.Subscriber('ObjectDetectnet', String, self.callbackObject)
        #self.subObject = rospy.Subscriber('ObjectYOLO', String, self.callbackObject)

        self.pubMsgDepth = rospy.Publisher('TPC3Depth', Float64, queue_size=1) 

    def callbackObject(self, msg):
        self.msgObject = ast.literal_eval(msg.data)
        if(self.msgDepth is not None and self.msgNave is not None and self.msgImage is not None):

            self.objectDepth = self.getDistances()
            self.pubMsgDepth.publish(self.objectDepth)  

    def callbackDepth(self, msg):
        self.msgDepth = self.bridge.imgmsg_to_cv2(msg,'32FC1')
     
    def callbackNave(self, msg):
        self.msgNave = self.bridge.imgmsg_to_cv2(msg,'32FC1')

    def callbackImagem(self, msg):
        self.msgImage = self.bridge.imgmsg_to_cv2(msg,'bgra8')

    #pega a distancia do ponto mais próximo
    def getDistances(self):
        distancia = 99 

        recorte = np.where(self.msgNave[:, :], self.msgDepth[:, :], math.inf)
        distancia_zed = round(np.nanmin(recorte), 2) 
        if(distancia_zed < 1.0 or distancia_zed > 99):
            distancia = distancia_zed

        else:
            for object in self.msgObject:
                self.msgObject[object]["distance"] = distancia
                bbox = self.msgObject[object]["coords"]
                
                rectangle = np.zeros((self.msgNave.shape[0], self.msgNave.shape[1]), dtype="float32")
                cv2.rectangle(rectangle, (bbox[0], bbox[1]), (bbox[2], bbox[3]), 255, -1)
                objectAhead = cv2.bitwise_and(self.msgNave, rectangle)
                
                if(objectAhead.any() > 0):
                    recorte = np.where(objectAhead[:, :], self.msgDepth[:, :], math.inf)
                    self.msgObject[object]["distance"] = round(np.nanmin(recorte), 2) 

                    if(distancia > self.msgObject[object]["distance"]):
                        distancia = self.msgObject[object]["distance"]
            
        return distancia
        
if __name__ == '__main__':
    try:
        node = NodeDepth()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Exception Depth: {}".format(rospy.ROSInterruptException))
        pass