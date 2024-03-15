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

# variável global para garantir que o tratamento da imagem so ira começar se tiver recebido imagem
flagLKAroi = False  
flagLKAresult = False
flagLKAnave = False

class NodeDashboard():

    def __init__(self):
        # Atributos ROS
        # Instancia um objeto do tipo bridge para fazer a conversão de opencv para msg_Image
        self.bridge = CvBridge()

        self.LKA_roi = None  # Recebera uma imagem.
        self.LKA_result = None
        self.LKA_nave = None

        self.subLKAroi = rospy.Subscriber('TPC6LKAroi', Image, self.callbackROI)
        self.subLKAresult = rospy.Subscriber('TPC7LKAresult', Image, self.callbackResult)
        self.subLKAnave = rospy.Subscriber('TPC8LKAnave', Image, self.callbackNave)  
    
    def callbackROI(self, msg):
        self.LKA_roi = self.bridge.imgmsg_to_cv2(msg, '8UC1')

        global flagLKAroi
        flagLKAroi = True  

    def callbackResult(self, msg):
        self.LKA_result = self.bridge.imgmsg_to_cv2(msg, '8UC3')

        global flagLKAresult
        flagLKAresult = True

    def callbackNave(self, msg):
        self.LKA_nave = self.bridge.imgmsg_to_cv2(msg, '8UC1')

        global flagLKAnave
        flagLKAnave = True  

class Dashboard():

    def working(self, imageROI, imageResult, imageNave):

        imageROI = cv2.resize(cv2.cvtColor(imageROI, cv2.COLOR_GRAY2BGR), (0, 0), None, .5, .5)
        imageNave = cv2.resize(cv2.cvtColor(imageNave, cv2.COLOR_GRAY2BGR), (0, 0), None, .5, .5)
        
        numpy_vertical = np.vstack((imageROI, imageNave))
        numpy_horizontal = np.hstack((imageResult, numpy_vertical))

        cv2.imshow('LIVE', numpy_horizontal)
        cv2.waitKey(1)


def main():
    # Setup ROS
    rospy.init_node('Dashboard')  # inicia o Node
    rospy.loginfo('O node Dashboard foi iniciado!')

    nodeDashboard = NodeDashboard()  # instanciando o objeto
    dashboard = Dashboard()
    while (not rospy.is_shutdown()):
        if (flagLKAroi and flagLKAresult and flagLKAnave):
            try:
                dashboard.working(
                    nodeDashboard.LKA_roi,
                    nodeDashboard.LKA_result, 
                    nodeDashboard.LKA_nave
                )

            except Exception as ex:
                print("Exception: {}".format(ex))
                pass


if (__name__ == "__main__"):
    main()
