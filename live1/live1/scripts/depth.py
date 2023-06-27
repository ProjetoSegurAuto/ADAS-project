#!/usr/bin/env python3

##########
#Informações sobre esse Node:
#Nome: depth
#Descrição: pega a profundidade da imagem.

import math
import numpy as np
#import vector as vc
import cv2
import rospy
from sensor_msgs.msg import Image       #tipo de mensagem para enviar imagem
from std_msgs.msg import Float64        #tipo de mensagem usada para enviar a distancia
from cv_bridge import CvBridge, CvBridgeError       #converter open cv para imagem ros

#variaveis globais

msgTCP2Deth = None          #recebe a matriz de profundidade
msgTPC1Camera = None        #recebe a imagem

flagImageReceived = False       #Confirma que recebeu a imagem, se não tiver a flag ele pode tratar uma variável nono e da merda
flagDepthReceived = False       #Confirma que recebeu a matriz de profundidade

class NodeDepth():

    def __init__(self):

        self.bridgeImg = CvBridge()             #Instancia um objeto do tipo bridge para fazer a conversão de opencv para msg_Image
        self.bridgeDepth = CvBridge()           #Instancia um objeto do tipo bridge para fazer a conversão de opencv para msg_Image    
        self.msgDepth = Float64()               #Instancia objeto do tipo Float64 para enviar o valor correto

        self.subImg = rospy.Subscriber('TPC1Camera', Image, self.callbackImagem)
        self.subDepth = rospy.Subscriber('TPC2Depth',Image, self.callbackDepth)

        self.pubDepth = rospy.Publisher('TPC3Depth',Float64,queue_size=1)
        
    #callback para a imagem
    def callbackImagem(self, msg_camera):
        global flagImageReceived            
        flagImageReceived = True            #Ativa a flag que indica o recebimento da mensagem
        global msgTPC1Camera
        msgTPC1Camera = self.bridgeImg.imgmsg_to_cv2(msg_camera,'bgra8')        #preenche a variavel global com a informacao

    #callback da profundidade
    def callbackDepth(self, msg_depth):
        global flagDepthReceived
        flagDepthReceived = True            #Ativa a flag que indica o recebimento da mensagem
        global msgTCP2Deth
        msgTCP2Deth = self.bridgeDepth.imgmsg_to_cv2(msg_depth,'32FC1')         #preenche a variavel global com a informacao

    #publica a profundidade
    def pubMsgDepth(self,msg_depth):
        self.msgDepth = msg_depth
        self.pubDepth.publish(self.msgDepth)

class Depth():
    def __init__(self):
        self.recorte = None             #receberar o recorte da matriz de profundidade

    #pega a distancia do ponto mais próximo
    def getDistanceMin(self,depth):
        self.recorte = depth[240:480,320:960].copy()         #Faz um recorte da image, analisa apenas uma parte da imagem (retangulo)
        distanciaMin = np.nanmin(self.recorte)               #Pega a menir idstancia sem ser um nan    

        #print area recortada, matriz de profundidade.
        # cv2.imshow('depthAre',recorte)
        # cv2.resizeWindow('depthAre',960,480)
        # cv2.waitKey(1)
        
        return distanciaMin
    
    #mostra  na imagem captura pela camera a area qeu vamos pegar a prodfundidade em forma de um retangulo
    def depthRangeView(self,img):
        sqrt_image = np.zeros_like(img)             #gera uma matriz do tamanho da imagem com zeros
        cv2.rectangle(sqrt_image,(320,240),(960,480),(255,0,0),10)          #cria um retangulo na imagem
        img = cv2.addWeighted(img, 0.97, sqrt_image, 0.5, 0)
        
        #print da imagem com retangulo
        cv2.imshow('depthRangeView', img)
        cv2.resizeWindow('depthRangeView', 1280, 720)
        cv2.waitKey(1)

def main():
    #Setup ROS
    rospy.init_node('Depth')                #inicia o Node
    rospy.loginfo('O node depth foi iniciado!')

    nodeDepth = NodeDepth()                 #instanciando o objeto do No ros
    depthobj = Depth()                      #instanciando o objeto da profundidade

    while not rospy.is_shutdown():          #Enquanto o ros não for fechado
        global flagImageReceived  
        global flagDepthReceived

        if (flagImageReceived and flagDepthReceived):           #analisa se as mensagens ja chegaram
            try:
                
                #chamada para a função, nao precisamos dela
                # global msgTPC1Camera
                # frame = msgTPC1Camera
                # depthobj.depthRangeView(frame)                

                global msgTCP2Deth
                depth = msgTCP2Deth
                distancia = depthobj.getDistanceMin(depth)
                nodeDepth.pubMsgDepth(distancia)          
            
            except Exception as e: 
                print(e)

if __name__ == "__main__":
    main()