#!/usr/bin/env python3

##########
# Informações sobre esse Node:
# Nome: Decision maker
# Descrição: A partir das infromações toma a decisão

import rospy
import math
import time
import vector as vc
import numpy as np
import socket
import asyncio
from threading import Thread
from std_msgs.msg import Float64, Float64MultiArray, String
import ast
import gc

HOST = "192.168.1.101"
PORT = 2323

flagSteeringReceived = False
flagCurveRadiusReceived = False
flagVehicleCanInit = False


class NodeDecisionMaker():
    def __init__(self):
        self.msgSteering = Float64()
        self.msgCurveRadius = Float64MultiArray()
       
        self.subSteering = rospy.Subscriber(
            'TPC4Steering', Float64, self.callbackSteering)
        self.subCurveRadius = rospy.Subscriber(
            'TPC5CurveRadius', Float64MultiArray, self.callbackCurveRadius)


    def callbackSteering(self, msg_steering):
        global flagSteeringReceived
        flagSteeringReceived = True
        self.msgSteering = int(msg_steering.data)

    def callbackCurveRadius(self, msg_radiusCurve):
        global flagCurveRadiusReceived
        flagCurveRadiusReceived = True
        self.msgCurveRadius = msg_radiusCurve.data

class DecisionMaker():
    def __init__(self):
        self.rpm_can = 40
        self.angle_can = 25
        self.tSendMsgCAN = time.time()
        self.angMin = 1
        self.angMax = 50
        self.timeMin = 0.005
        self.socket = vc.openSocket()

    def initCar(self):
        print("Inicio do carro autorizado!")
        try:
            msgCanId = 0x56
            param = [1, self.rpm_can, 1, self.rpm_can]
            vc.sendMsg(self.socket, msgCanId, param)
            #vc.logCAN(self.socket)
            print("Mensagem para ECU POWERTRAIN: ", msgCanId, param)

            msgCanId = 0x82
            param = [self.angle_can]
            vc.sendMsg(self.socket, msgCanId, param)
            #vc.logCAN(self.socket)
            print("Mensagem para ECU DIREÇÃO: ", msgCanId, param)
            
        except Exception as ex:
            print("Falha ao inicia carro: {}".format(ex))


async def main_async():
    global flagVehicleCanInit

    rospy.init_node("Decision_maker")
    print("O node Decision maker foi iniciado")

    nodeDecisionMaker = NodeDecisionMaker()
    dm = DecisionMaker()
    s = dm.socket

    while not flagVehicleCanInit:
        if  flagCurveRadiusReceived and flagSteeringReceived:
            flagVehicleCanInit = True 
            # modificar função initCar para receber os parametros da recInfoSoftware e iniciar o veiculo
            dm.initCar()

    # print(s)
    while not rospy.is_shutdown():

        try:
            if dm.timeMin < time.time() - dm.tSendMsgCAN:
                dm.tSendMsgCAN = time.time()
                angDir = nodeDecisionMaker.msgSteering

                if angDir < dm.angMin:
                    angDir = dm.angMin
                elif angDir > dm.angMax:
                    angDir = dm.angMax

                msgCanId = 0x82
                param = [angDir]
                vc.sendMsg(s, msgCanId, param)
    
                gc.collect()

        except Exception as ex:
            print("Exception: {}".format(ex))
            pass

        except KeyboardInterrupt:
            print("Exception: KeyboardInterrupt")
            angDir = 25
            rpm = 0
            
            msgCanId = 0x56
            param = [1, rpm, 1, rpm]
            vc.sendMsg(s, msgCanId, param)

            msgCanId = 0x82
            param = [angDir]
            vc.sendMsg(s, msgCanId, param)
            s.close()
            break

        except:
            print("F")


def main():
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main_async())


if __name__ == '__main__':
    main()
