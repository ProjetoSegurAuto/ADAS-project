#!/usr/bin/env python3

#################
#Informações sobre esse Node:
#Nome: nodeSign
#Descrição: Recebe a image da camera zed e retorna as coordenadas dos sinais de transito detectados

import rospy
import cv2
import numpy as np         
from sensor_msgs.msg import Image                   #tipo de mensagem para enviar imagem
from std_msgs.msg import Float64                    #tipo de mensagem que sera enviado
from std_msgs.msg import Float64MultiArray          #tipo de mensagem que sera enviado
from cv_bridge import CvBridge, CvBridgeError       #converter open cv para imagem ros
import gc
from ultralytics import YOLO

#variável global para garantir que o tratamento da imagem so ira começar se tiver recebido imagem
flagImageReceived = False           

class NodeSign():

    def __init__(self):
        #Atributos ROS
        ##Recebe uma imagem.
        self.msgImage = Image()   
        ##Instancia um objeto do tipo bridge para fazer a conversão de opencv para msg_Image               
        self.bridge = CvBridge()       
        ##Instancia um objeto do tipo float que servira para enviar as coordenadas dos objetos       
        self.msgXYXYobject = Float64MultiArray()  

        #atributos saida
        self.msgSaida = None

        #recebimento de mensagens
        self.subImage = rospy.Subscriber('TPC1Camera', Image, self.callbackImage)
        #publicação 
        self.pubXYXYobject = rospy.Publisher('TPC6ObjectCoordinates', Float64MultiArray, queue_size=1)

    #callback para a imagem
    def callbackImage(self, msg_image):
        global flagImageReceived            
        flagImageReceived = True
        #talvez tenha que usar uma variaǘel global apra receber a imagem
        self.msgImage = self.bridge.imgmsg_to_cv2(msg_image,'bgra8')

class Sign():
    def __init__(self):
        self.model = YOLO("best_93.pt")   

    def getCoordinates(self, image):
        results = self.model.predict(image[:, :, :3])
        #results = model.predict(image)
        output = results[0].plot()
        #cv2.imshow("result", res_plotted)

        return output

def main():
    global flagImageReceived 
    #Inicia o node do ROS
    rospy.init_node('signDetection')
    rospy.loginfo('O node sinais de transito foi iniciado!')

    #Instancia o objeto do node
    nodesign = NodeSign()

    sign = Sign()

    #loop de operação:
    while(not rospy.is_shutdown()):
        if(flagImageReceived):
            try:
                #Passa a imagem recebida para o frame
                frame = nodesign.msgImage             

                output = sign.getCoordinates(frame)

                #cv2.imshow('Depth',depth)
                #cv2.imshow('SignDetection', frame)
                cv2.imshow('resultado',output)
                cv2.waitKey(1)
                gc.collect()
            except Exception as ex:
                print("Exception: {}".format(ex))
                pass

    rospy.spin()

if(__name__=='__main__'):
    main()