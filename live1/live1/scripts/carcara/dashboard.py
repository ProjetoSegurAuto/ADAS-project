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
flagImgYolo = False
flagImgDetectNet = False


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
        self.subImgYOLO = rospy.Subscriber('TPC3ImgYOLO', Image, self.callbackYOLO)  # inscrição no topico de YOLO
        
        #self.subImgDetectNet = rospy.Subscriber('detections_with_boxes', Image, self.callbackDetectNet)

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

    def callbackYOLO(self, msg_img_yolo):

        # quando a imagem for recebida ela será convertida de msgImage para objeto do opencv
        self.msgTPC3ImgYOLO = self.bridge.imgmsg_to_cv2(msg_img_yolo, '8UC3')

        global flagImgYolo
        # Como a mensagem chegou, ativa a flag que permite o tratamento da imagem
        flagImgYolo = True

    def callbackDetectNet(self, msg_img_detectnet):

        # quando a imagem for recebida ela será convertida de msgImage para objeto do opencv
        self.msgDetectNet = self.bridge.imgmsg_to_cv2(msg_img_detectnet, "bgra8")

        global flagImgDetectNet
        # Como a mensagem chegou, ativa a flag que permite o tratamento da imagem
        flagImgDetectNet = True

class Dashboard():

    def working(self, imageROI, imageResult, imageYolo):

        # image = image[:, :, :3]

        # I just resized the image to a quarter of its original size
        imageROI = cv2.resize(cv2.cvtColor(imageROI, cv2.COLOR_GRAY2BGR), (0, 0), None, .5, .5)#cv2.resize(imageROI, (0, 0), None, .5, .5)
        imageYolo = cv2.resize(imageYolo, (0, 0), None, .5, .5)
        #imageDetectNet = cv2.resize(imageDetectNet, (0, 0), None, .5, .5)
        #imageResult = cv2.resize(imageResult, (0, 0), None, .75, .75)

        # grey = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        # Make the grey scale image have three channels
        # grey_3_channel = cv2.cvtColor(grey, cv2.COLOR_GRAY2BGR)

        numpy_vertical = np.vstack((imageROI, imageYolo))
        numpy_horizontal = np.hstack((imageResult, numpy_vertical))

        #numpy_vertical_concat = np.concatenate((imageROI, imageResult, imageYolo), axis=0)

        #numpy_horizontal_concat = np.concatenate((imageROI, imageResult, imageYolo), axis=1)

        # cv2.imshow('Main', image)
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
        if (flagLKAroi and flagLKAresult and flagImgYolo):
            try:
                dashboard.working(
                    nodeDashboard.msgTPC6LKAroi,
                    nodeDashboard.msgTPC7LKAresult, 
                    nodeDashboard.msgTPC3ImgYOLO
                )

            except Exception as ex:
                #print("Exception: {}".format(ex))
                pass

            #except KeyboardInterrupt:
            #    print("Exception: KeyboardInterrupt Lane")


if (__name__ == "__main__"):
    main()
