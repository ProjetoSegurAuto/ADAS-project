#!/usr/bin/env python3

##########
# Informações sobre esse Node:
# Nome: Dashboard
# Descrição: Exibe imagens do processo do ADAS (LKA ROI, LKA Resultado, YOLO)

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image  # tipo de mensagem para enviar imagem
# converter open cv para imagem ros
from cv_bridge import CvBridge, CvBridgeError

flagLKAroi = False  # variável global para garantir que o tratamento da imagem so ira começar se tiver recebido imagem
flagLKAresult = False
flagImgObject = False


class NodeDashboard():

    def __init__(self):
        # Atributos ROS

        self.msgTPC6LKAroi = None  # Recebera uma imagem.
        self.msgTPC7LKAresult = None
        # Instancia um objeto do tipo bridge para fazer a conversão de opencv para msg_Image
        self.bridge = CvBridge()

        # self.sub = rospy.Subscriber('TPC1Camera',Image,self.callback)       #inscrição no topico de imagens
        # inscrição no topico de LKA ROI
        self.subLKAroi = rospy.Subscriber('TPC6LKAroi', Image, self.callbackROI)
        # inscrição no topico de LKA resultado
        self.subLKAresult = rospy.Subscriber('TPC7LKAresult', Image, self.callbackResult)
        #self.subImgObject = rospy.Subscriber('ImgYOLO', Image, self.callbackObject)  # inscrição no topico de YOLO
        self.subImgObject = rospy.Subscriber('ImgDetectnet', Image, self.callbackObject) 

    def callbackROI(self, msg_roi):
        # quando a imagem for recebida ela será convertida de msgImage para objeto do opencv
        self.msgTPC6LKAroi = self.bridge.imgmsg_to_cv2(msg_roi, '8UC1')

        global flagLKAroi
        flagLKAroi = True  # Como a mensagem chegou, ativa a flag que permite o tratamento da imagem

    def callbackResult(self, msg_result):
        # quando a imagem for recebida ela será convertida de msgImage para objeto do opencv
        self.msgTPC7LKAresult = self.bridge.imgmsg_to_cv2(msg_result, '8UC3')

        global flagLKAresult
        # Como a mensagem chegou, ativa a flag que permite o tratamento da imagem
        flagLKAresult = True

    def callbackObject(self, msg_img):
        # quando a imagem for recebida ela será convertida de msgImage para objeto do opencv
        self.msgImgObject = self.bridge.imgmsg_to_cv2(msg_img, '8UC3')

        global flagImgObject
        # Como a mensagem chegou, ativa a flag que permite o tratamento da imagem
        flagImgObject = True

class Dashboard():

    def working(self, imageROI, imageResult, imageObject):
        imageROI = cv2.resize(cv2.cvtColor(imageROI, cv2.COLOR_GRAY2BGR), (0, 0), None, .5, .5)#cv2.resize(imageROI, (0, 0), None, .5, .5)
        imageObject = cv2.resize(imageObject, (0, 0), None, .5, .5)

        numpy_vertical = np.vstack((imageROI, imageObject))
        numpy_horizontal = np.hstack((imageResult, numpy_vertical))

        '''
        cv2.imshow('Numpy Vertical', numpy_vertical)
        cv2.imshow('Numpy Horizontal', numpy_horizontal)
        cv2.imshow('Numpy Vertical Concat', numpy_vertical_concat)
        cv2.imshow('Numpy Horizontal Concat', numpy_horizontal_concat)
        '''

        cv2.imshow('LIVE', numpy_horizontal)
        cv2.waitKey(1)


def main():
    # Setup ROS
    rospy.init_node('Dashboard')  # inicia o Node
    rospy.loginfo('O node Dashboard foi iniciado!')

    nodeDashboard = NodeDashboard()  # instanciando o objeto
    dashboard = Dashboard()

    while (not rospy.is_shutdown()):
        if (flagLKAroi and flagLKAresult and flagImgObject):
            try:
                dashboard.working(
                    nodeDashboard.msgTPC6LKAroi,
                    nodeDashboard.msgTPC7LKAresult, 
                    nodeDashboard.msgImgObject
                )

            except Exception as ex:
                print("Exception Dashboard: {}".format(ex))
                pass

if (__name__ == "__main__"):
    main()
