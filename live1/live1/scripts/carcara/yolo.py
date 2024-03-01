#!/usr/bin/env python3

#################
#Informações sobre esse Node:
#Nome: nodeYOLO
#Descrição: Recebe a image da camera zed e retorna as coordenadas dos objetos detectados

import rospy
import cv2
import numpy as np         
from sensor_msgs.msg import Image                   #tipo de mensagem para enviar imagem
from std_msgs.msg import Float64MultiArray, String  #tipo de mensagem que sera enviado
from cv_bridge import CvBridge, CvBridgeError       #converter open cv para imagem ros
import gc                                           #garbage collector
from ultralytics import YOLO                        #YOLO   

class NodeYOLO():
    def __init__(self):
        rospy.init_node('yolo')
        rospy.loginfo('O node YOLO foi iniciado!')
        
        #Atributos ROS
        ##Carrega o modelo
        self.model = YOLO("yolov8n.pt")  
        ##Recebe uma imagem
        self.image = Image()       
        ##Instancia um objeto do tipo bridge para fazer a conversão de opencv para image               
        self.bridge = CvBridge()      
        ##Instancia um objeto do tipo float que servira para enviar as coordenadas dos objetos       
        self.objectYOLO = String()  

        #recebimento de mensagens
        self.subImage = rospy.Subscriber('TPC1Camera', Image, self.callbackImage)

        #publicação 
        self.pubImgYOLO = rospy.Publisher('ImgYOLO', Image, queue_size=1)           #guarda os parametros para o envio da imagem da yolo
        self.pubObjectYOLO = rospy.Publisher('ObjectYOLO', String, queue_size=1)         #guarda os parametros para o envio das informações da yolo

    #callback para a imagem
    def callbackImage(self, image):
        #talvez tenha que usar uma variável global apra receber a imagem
        self.image = self.bridge.imgmsg_to_cv2(image,'bgra8')     

        self.objectYOLO.data, imageYOLO = self.getObjetcs(self.image[:, :, :3])

        self.pubObjectYOLO.publish(self.objectYOLO)

        self.msgImgYOLO = self.bridge.cv2_to_imgmsg(imageYOLO,"8UC3")
        self.pubImgYOLO.publish(self.msgImgYOLO)   
    
    #Detecta objetos na imagem
    def getObjetcs(self, image):
        results = self.model.predict(image, conf=0.2, verbose=False)  #0.7
        objectsYOLO = {}
        objectYOLO = {
            "classId": 0,
            "cords": [0, 0, 0, 0],
            "conf": 0,
            "distance": 0
        }
        #comentar caso não queira debugar
        imageYOLO = image
        if(results[0]):
            result = results[0]
            objectYOLOID = 0
            #comentar caso não queira debugar
            imageYOLO = result.plot() 
            for box in result.boxes:
                objectYOLO["classId"]  = result.names[box.cls[0].item()]
                objectYOLO["coords"] = box.xyxy[0].tolist()
                objectYOLO["coords"] = [round(x) for x in objectYOLO["coords"]]
                objectYOLO["conf"] = round(box.conf[0].item(), 2)
                #objectYOLO["distance"] = np.Inf
                objectYOLOID = objectYOLOID + 1
                objectsYOLO[str(objectYOLOID)] = objectYOLO
                #comentar caso não queira debugar
                #print("Objeto Yolo ({}):\n-classId: {}\n-coords: {}\n-conf: {}\n-depth: {}\n\n".format(object, objectsYOLO[str(objectYOLOID)]['classId'], objectsYOLO[str(objectYOLOID)]['coords'], objectsYOLO[str(objectYOLOID)]['conf'], objectsYOLO[str(objectYOLOID)]['distance']))
        return str(objectsYOLO), imageYOLO

if __name__ == '__main__':
    try:
        node = NodeYOLO()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Exception: {}".format(rospy.ROSInterruptException))