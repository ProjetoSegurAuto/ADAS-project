#!/usr/bin/env python3

#################
#Informações sobre esse Node:
#Nome: nodePlates
#Descrição: Pegar a imagem da camera zed identifica placas

import rospy
import cv2
import numpy as np              
from sensor_msgs.msg import Image                   #tipo de mensagem para enviar imagem
from std_msgs.msg import Float64                    #tipo de mensagem que sera enviado
from std_msgs.msg import Float64MultiArray          #tipo de mensagem que sera enviado
from cv_bridge import CvBridge, CvBridgeError       #converter open cv para imagem ros

flagImageReceived = False           #variável global para garantir que o tratamento da imagem so ira começar se tiver recebido imagem
flagDepthReceived = False           #variável global para garantir que o tratamento do depth so ira começar se tiver recebido o depth

class NodePlates():

    def __init__(self):
        #atributos entrada
        self.msgImage = Image()               #Recebe a image tem que ter o formato da mensagem que vai receber
        self.imageBridge = CvBridge()       #converte a msg_image_ros para open cv
        self.msgDepth = Image()                #Recebe a depth tem que ter o formato da mensagem que vai receber
        self.depthBridge = CvBridge()       #converte a msg_image_ros para open cv

        #atributos saida
        self.msgSaida = Nonecv2_to_imgmsgber('TPC1Camera',Image,self.callbackImage)
        self.subDepth = rospy.Subscriber('TPC2Depth',Image,self.callbackDepth)

        #publicação de imagens
        self.pubSaida = rospy.Publisher('TPC6Plates',Float64,queue_size=1)

    #callback para a imagem
    def callbackImage(self,msg_image):
        global flagImageReceived            
        flagImageReceived = True
        #talvez tenha que usar uma variaǘel global apra receber a imagem
        self.msgImage = self.imageBridge.imgmsg_to_cv2(msg_image,'bgra8')

    #callback para a prodfundidade
    def callbackDepth(self, msg_depth):
        global flagDepthReceived
        flagDepthReceived = True
        #talvez tenha que usar uma variaǘel global apra receber depth
        self.msgDepth = self.depthBridge.imgmsg_to_cv2(msg_depth,'32FC1')

    #publica a saida
    # def pubSaida(self, msg_saida):
    #     self.msgSaida = msg_saida
    #     self.pubSaida.publish(self.msgSaida)

class Plates():
    def __init__(self):
        #atributos
        self.distancia = []            #guarda as informações para das distancias

    def getDistance(self,points,depth):
        print('Get distance')
        lisrPoints=points.astype(int).copy()           #convert para inteiro, isso é importante
        distanciaMin = []

        for points in lisrPoints:
            x = []
            y = []
            for row in points:

                x.append(row[0])
                y.append(row[1])

            #print(points)
            xmin = min(x)
            xmax = max(x)
            ymin = min(y)
            ymax = max(y)

            recorte = depth[ymin:ymax,xmin:xmax].copy()     #Faz um recorte da image, analisa apenas uma parte da imagem (retangulo)
            distanciaMin.append(np.nanmin(recorte))               #Pega a menor idstancia sem ser um nan

        print(distanciaMin)  
        
        # print("xmin: ",xmin)
        # print("xmax: ",xmax)
        # print("ymin: ",ymin)
        # print("ymax: ",ymax)

        #1  2
        #3  4

        #   x1      y1
        # [[ 978   45]
        #   x2      y2
        # [1232   29]
        #   x3      y3
        # [1223  278]
        #   x4      y3
        # [ 978  285]]

        # print("Pegar a distancia")
        # print(type(points))
        # print(points)

        # for index,area in points.iterrows():
        #     self.distancia[index]= area
        #     print(self.distancia[index])

def main():
    #inicia o node do ROS
    rospy.init_node('placas')
    rospy.loginfo('O node placas foi iniciado!')

    #instancia o objeto do node
    nodeplates = NodePlates()

    plates = Plates()

    #loop de operação:
    while(not rospy.is_shutdown()):

        if(flagImageReceived & flagDepthReceived):
            try:
                
                frame = nodeplates.msgImage             #passa a imagem recebida para o frame
                depth = nodeplates.msgDepth             #passa a depth recebida para o depth

                qcd = cv2.QRCodeDetector()              #instancia um objeto para

                retval, decoded_info, points, straight_qrcode = qcd.detectAndDecodeMulti(frame)         #detecta e decodifica multiplos qrCodes

                if(retval):         #Se a camera encontrar um qr code na imagem
                    print(decoded_info)
                    plates.getDistance(points,depth)        #printa os pontos do qrcode

                    img = cv2.polylines(frame, points.astype(int), True, (0, 255, 0), 3)       #Desenha uma caixa no ao redor do qrcode
            
                    frame = cv2.addWeighted(frame, 1.0, img, 0.5, 0)                #sobrepõe as imagens
                
                #cv2.imshow('Depth',depth)
                cv2.imshow('Placas',frame)
                cv2.waitKey(1)
            except:
                pass

    rospy.spin()

if(__name__=='__main__'):
    main()