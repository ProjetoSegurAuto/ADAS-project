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

flagDistanceReceived = False
flagVehiclePositionReceived = False
flagSteeringReceived = False
flagCurveRadiusReceived = False
flagObjectYOLOReceived = False
flagVehicleCanInit = False
flagQRCode = False


class NodeDecisionMaker():
    def __init__(self):
        self.msgDepth = Float64()
        self.msgVehiclePosition = Float64()
        self.msgSteering = Float64()
        self.msgCurveRadius = Float64MultiArray()
        self.msgObjectYOLO = String()
        self.msgQRCode = Float64()

        self.subDepth = rospy.Subscriber(
            'TPC3Depth', Float64, self.callbackDepth)
        self.subVehiclePosition = rospy.Subscriber(
            'TPC4VehiclePosition', Float64, self.callbackVehiclePosition)
        self.subSteering = rospy.Subscriber(
            'TPC4Steering', Float64, self.callbackSteering)
        self.subCurveRadius = rospy.Subscriber(
            'TPC5CurveRadius', Float64MultiArray, self.callbackCurveRadius)
        self.subObjectYOLO = rospy.Subscriber(
            'TPC3ObjectYOLO', String, self.callbackObjectYOLO)
        self.subQRCode = rospy.Subscriber(
            'TPC6QRCode', Float64, self.callbackQRCode)

    def callbackDepth(self, msg_depth):
        global flagDistanceReceived
        flagDistanceReceived = True
        self.msgDepth = msg_depth.data

    def callbackVehiclePosition(self, msg_vehiclePosition):
        global flagVehiclePositionReceived
        flagVehiclePositionReceived = True
        self.msgVehiclePosition = msg_vehiclePosition.data

    def callbackSteering(self, msg_steering):
        global flagSteeringReceived
        flagSteeringReceived = True
        self.msgSteering = int(msg_steering.data)

    def callbackCurveRadius(self, msg_radiusCurve):
        global flagCurveRadiusReceived
        flagCurveRadiusReceived = True
        self.msgCurveRadius = msg_radiusCurve.data

    def callbackObjectYOLO(self, msg_objectYOLO):
        global flagObjectYOLOReceived
        flagObjectYOLOReceived = True
        self.msgObjectYOLO = msg_objectYOLO

    def callbackQRCode(self, msg_QRCode):
        global flagQRCode
        flagQRCode = True
        self.msgQRCode = msg_QRCode


class DecisionMaker():
    def __init__(self):
        self.rpm_can = 0
        self.angle_can = 25
        self.distanceBreak = 1.2
        self.distanceStop = 0.5
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
            print("Mensagem para ECU POWERTRAIN: ", msgCanId, param)

            msgCanId = 0x82
            param = [self.angle_can]
            vc.sendMsg(self.socket, msgCanId, param)
            print("Mensagem para ECU DIREÇÃO: ", msgCanId, param)
        except Exception as ex:
            print("Falha ao inicia carro: {}".format(ex))

    def AEB(self, distance):
        retorno = False
        msgCanId = 0x00
        try:
            if not np.isnan(distance):
                if np.isfinite(distance):
                    if distance < self.distanceStop:
                        print("STOP!!! - distance: {}".format(distance))
                        msgCanId = 0x5C
                        retorno = True
                    elif distance < self.distanceBreak:
                        print("Break!!! - distance: {}".format(distance))
                        msgCanId = 0x56
                        retorno = True
                else:
                    print("STOP!!! infinite - distance: {}".format(distance))
                    msgCanId = 0x5C
                    retorno = True

                param = [1, 0, 1, 0]
                vc.sendMsg(self.socket, msgCanId, param)
                return retorno

        except Exception as ex:
            print("Exception: {}".format(ex))
            return retorno

    # sempre enviando as mesagens para o software Grojoba
    def sendInfoSotftware(self, IDcar=2,):
        msgCanId = 0x94
        param = [IDcar,]
        vc.sendMsg(self.socket, msgCanId, param)

    # Recebe parametros iniciais do veiculo
    def recInfoSoftware(self,):
        vc.logCanPlatoon()


async def main_async():
    global flagVehicleCanInit
    global flagQRCode
    global flagObjectYOLOReceived

    rospy.init_node("Decision_maker")
    print("O node Decision maker foi iniciado")

    nodeDecisionMaker = NodeDecisionMaker()
    dm = DecisionMaker()
    s = dm.socket
    start = True

    while not flagVehicleCanInit:
        if flagVehiclePositionReceived and flagCurveRadiusReceived and flagDistanceReceived and flagSteeringReceived:
            flagVehicleCanInit = True and flagObjectYOLOReceived == False
            # modificar função initCar para receber os parametros da recInfoSoftware e iniciar o veiculo
            dm.initCar()

    flagQRCode = True
    while not flagQRCode:
        print('Flag QR Code: {}'.format(flagQRCode))
        if flagVehiclePositionReceived and flagCurveRadiusReceived and flagDistanceReceived and flagSteeringReceived:
            flagQRCode = True

    # print(s)
    while not rospy.is_shutdown():

        try:
            if dm.timeMin < time.time() - dm.tSendMsgCAN:
                dm.tSendMsgCAN = time.time()
                angDir = nodeDecisionMaker.msgSteering

                # recebe os parametros inciais definidos no codigo, depois do primiero inicio o software controla os parametros do veiculo
                if start == True:
                    rpm = dm.rpm_can
                    start = False
                else:
                    msgPlatoon, rpmDir, rpmEsq = dm.recInfoSoftware()
                    rpm = rpmDir

                if angDir < dm.angMin:
                    angDir = dm.angMin
                elif angDir > dm.angMax:
                    angDir = dm.angMax

                msgCanId = 0x82
                param = [angDir]
                vc.sendMsg(s, msgCanId, param)

                breaking = dm.AEB(nodeDecisionMaker.msgDepth)
                # sempre enviar msg para o software
                dm.sendInfoSotftware(angDir)

                if not breaking:
                    msgCanId = 0x56
                    param = [1, rpm, 1, rpm]
                    vc.sendMsg(s, msgCanId, param)
                    # sempre enviar msg para o software
                    dm.sendInfoSotftware(angDir)
                    # vc.logCAN(s)
                else:
                    pass

                if flagObjectYOLOReceived == True:
                    jsonObjectYOLO = ast.literal_eval(
                        nodeDecisionMaker.msgObjectYOLO.data)

                    for object in jsonObjectYOLO:
                        if (jsonObjectYOLO[object]['classId'] == "stop sign"):
                            print('Placa de PARE identificada \n Parando...')
                            msgCanId = 0x56
                            # [Direcao Esq, RPM Esq, #Direcao Dir, RPM Dir]
                            param = [1, 0, 1, 0]
                            vc.sendMsg(msgCanId, param)

                gc.collect()

        except Exception as ex:
            print("Exception: {}".format(ex))
            pass

        except KeyboardInterrupt:
            print("Exception: KeyboardInterrupt")
            msgCanId = 0x56
            param = [1, 0, 1, 0]
            vc.sendMsg(s, msgCanId, param)

            msgCanId = 0x82
            param = [0x19]
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
