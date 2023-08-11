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

#variável global para garantir que o tratamento da imagem so ira começar se tiver recebido imagem
flagImageReceived = False    
flagDepthReceived = False       

class NodeYOLO():
    def __init__(self):
        #Atributos ROS
        ##Carrega o modelo
        self.model = YOLO("yolov8n.pt")  
        ##Recebe uma imagem
        self.image = Image()      
        self.depth = Image()      
        ##Instancia um objeto do tipo bridge para fazer a conversão de opencv para image               
        self.bridge = CvBridge()   
        self.bridgeDepth = CvBridge()     
        ##Instancia um objeto do tipo float que servira para enviar as coordenadas dos objetos       
        self.objectYOLO = String()  

        #recebimento de mensagens
        self.subImage = rospy.Subscriber('TPC1Camera', Image, self.callbackImage)
        self.subImage = rospy.Subscriber('TPC2Depth', Image, self.callbackDepth)

        #publicação 
        self.pubImgYOLO = rospy.Publisher('TPC3ImgYOLO', Image, queue_size=1)           #guarda os parametros para o envio da imagem da yolo
        self.pubObjectYOLO = rospy.Publisher('TPC3ObjectYOLO', String, queue_size=1)         #guarda os parametros para o envio das informações da yolo

    #callback para a imagem
    def callbackImage(self, image):
        global flagImageReceived            
        flagImageReceived = True
        #talvez tenha que usar uma variável global apra receber a imagem
        self.image = self.bridge.imgmsg_to_cv2(image,'bgra8')
    
    #callback da profundidade
    def callbackDepth(self, msg_depth):
        global flagDepthReceived
        flagDepthReceived = True
        #preenche a variável global com a informação
        self.depth = self.bridgeDepth.imgmsg_to_cv2(msg_depth,'32FC1')         
    
    #Detecta objetos na imagem
    def getObjetcs(self, image):
        results = self.model.predict(image, conf=0.7)  
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
                objectYOLO["distance"] = round(self.getDistance(objectYOLO["coords"]), 2)
                objectYOLOID = objectYOLOID + 1
                objectsYOLO[str(objectYOLOID)] = objectYOLO
                #comentar caso não queira debugar
                #print("Objeto Yolo ({}):\n-classId: {}\n-coords: {}\n-conf: {}\n-depth: {}\n\n".format(object, objectsYOLO[str(objectYOLOID)]['classId'], objectsYOLO[str(objectYOLOID)]['coords'], objectsYOLO[str(objectYOLOID)]['conf'], objectsYOLO[str(objectYOLOID)]['distance']))
        return str(objectsYOLO), imageYOLO

    def getDistance(self, coords):
        x1, y1, x2, y2 = coords
        recorte = self.depth[y1:y2, x1:x2].copy() #Faz um recorte da image, analisa apenas uma parte da imagem (retangulo)
        distanciaMin = np.nanmin(recorte)         #Pega a menir idstancia sem ser um nan    
        if(distanciaMin < 0):
            distanciaMin = 0
        
        return distanciaMin

def main():
    global flagImageReceived 
    #Inicia o node do ROS
    rospy.init_node('yolo')
    rospy.loginfo('O node YOLO foi iniciado!')

    #Instancia o objeto do node
    yolo = NodeYOLO()

    #loop de operação:
    while(not rospy.is_shutdown()):
        if(flagImageReceived):
            try:
                #Passa a imagem recebida para o frame
                frame = yolo.image             
                yolo.objectYOLO.data, imageYOLO = yolo.getObjetcs(frame[:, :, :3])

                #comentar caso não queira debugar
                #cv2.imshow('YOLO', imageYOLO)
                #cv2.waitKey(1)

                yolo.pubObjectYOLO.publish(yolo.objectYOLO)

                yolo.msgImgYOLO = yolo.bridge.cv2_to_imgmsg(imageYOLO,"8UC3")
                yolo.pubImgYOLO.publish(yolo.msgImgYOLO) 

                gc.collect()
            except Exception as ex:
                print("Exception: {}".format(ex))
                #break

    rospy.spin()

if(__name__=='__main__'):
    main()