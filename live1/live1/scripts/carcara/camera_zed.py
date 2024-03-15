#!/usr/bin/env python3

##########
#Informações sobre esse Node:
#Nome: Camera
#Descrição: Abrir a camera zed e enviar as imagens no Topico imagens

import rospy
import cv2
import numpy as np
import pyzed.sl as sl   #biblioteca da camerazed
from sensor_msgs.msg import Image   #tipo de mensagem para enviar imagem
from cv_bridge import CvBridge, CvBridgeError #converter um objeto do open cv para mensagem imagem padrão do ros

#Aqui é definido a classe desse node
class NodeCamera():

    def __init__(self):

        #Atributos ROS
        self.bridge = CvBridge()    #Instancia um objeto do tipo bridge para fazer a conversão de opencv para msg_Image
        self.msgImagem = Image()    #guarda a mensagem do tipo imagem que sera enviada
        self.msgDepth = Image()     #guarda a mensagem do tipo imagem com a profundidade que sera enviada

        self.pubImagem = rospy.Publisher('TPC1Camera',Image,queue_size=1)   #guarda os parametros para o envio da imagem
        self.pubDepth = rospy.Publisher('TPC2Depth', Image,queue_size=1)     #guarda os parametros para o envio da imagem com profundidade

        #Atributos Parametros ZED CAMERA
        self.cam = sl.Camera()                  #objeto camera, sera usado para abrir a camera
        self.init = sl.InitParameters()         #objeto para setar os parametros da camera
        self.runtime = sl.RuntimeParameters()   #objeto com os parametros de Runtime

        #Atributos para Captura imagem e profundidade
        self.image = sl.Mat()       #guarda a imagem
        self.depth = sl.Mat()       #guarda a profundidade

        self.setupCameraZed()

    #metodo para setar os parametros da camera zed
    def setupCameraZed(self):

        self.init.camera_resolution = sl.RESOLUTION.HD720          
        self.init.camera_fps = 30
        self.init.depth_mode = sl.DEPTH_MODE.ULTRA 
        self.init.coordinate_units = sl.UNIT.METER  
        self.init.coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        
        #Abertura da camera
        if not self.cam.is_opened():
            print("ZED Camera opened.")

        status = self.cam.open(self.init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            exit()

    #entra em mode de operação
    def nodeWorking(self):
        while not rospy.is_shutdown():

            err = self.cam.grab(self.runtime)
            
            #Captura a imagem
            self.cam.retrieve_image(self.image, sl.VIEW.LEFT)
            #Captura a profundidade
            self.cam.retrieve_measure(self.depth, sl.MEASURE.DEPTH)

            #Converte a imagem para um formato que pode ser enviado
            self.msgImagem = self.bridge.cv2_to_imgmsg(self.image.get_data(),"bgra8")
            self.pubImagem.publish(self.msgImagem)

            #Converte a prodfundidade para um formato que pode ser enviado
            self.msgDepth=self.bridge.cv2_to_imgmsg(self.depth.get_data(),"32FC1")
            self.pubDepth.publish(self.msgDepth)

def main():
    
    #Setup ROS
    rospy.init_node('Camera') #inicia o Node
    rospy.loginfo('O node Camera foi iniciado!')

    nodeCamera = NodeCamera() #instanciando o objeto
    nodeCamera.nodeWorking()

if __name__ == '__main__':
    main()