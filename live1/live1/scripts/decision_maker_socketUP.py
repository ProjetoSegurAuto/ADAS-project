#!/usr/bin/env python3

##########
# Informações sobre esse Node:
# Nome: Decision maker
# Descrição: A partir das infromações toma a decisão

import rospy
import time
import vector as vc
import numpy as np
import asyncio
from threading import Thread
from std_msgs.msg import Float64, Float64MultiArray, String
import ast
import gc

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

        self.socket = vc.openSocket()  #Inicializa o socket de comunicação com a CAN

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
        breakingAEB = False
        breakingAEB = self.AEB(msg_depth.data)
        self.msgDepth = breakingAEB
    
    def AEB(self, distance):
        aeb = False
        msgCanId = 0x00
        param = []
        distanceBreak = 1.6#1.2       #Define a distância que o freio de emergência será acionado
        distanceStop = 0.9 #0.5       #Define a distância que o carro vai parar

        if not np.isnan(distance):
            if np.isfinite(distance):
                if distance < distanceStop:
                    print("TRAVA RODA - FREIA PWM - Distância: {}".format(distance))
                    msgCanId = 0x5C
                    aeb = True
                elif distance < distanceBreak:
                    print("FREIA PID - Distância: {}".format(distance))
                    msgCanId = 0x56
                    aeb = True
            else:
                print("TRAVA RODA - FREIA PWM - Distância: {}".format(distance))
                msgCanId = 0x5C
                aeb = True

        if aeb:     
            self.Powertrain(msgCanId, 0, 0)
            vc.sendMsg(self.socket, msgCanId, param)

        return aeb

    def Powertrain(self, msgCanId, rpmDir, rpmEsq):
        param = [1, rpmDir, 1, rpmEsq]
        vc.sendMsg(self.socket, msgCanId, param)
        print("Mensagem para ECU POWERTRAIN: {}; {}".format(msgCanId, param))

    def Direcao(self, angle):
        msgCanId = 0x82
        param = [angle]
        vc.sendMsg(self.socket, msgCanId, param)
        print("Mensagem para ECU DIREÇÃO:  {}; {}".format(msgCanId, param))

    def initCar(self, rpmDir, rpmEsq, angle):
        print("Inicialização do carro autorizada!")
        try:
            self.Powertrain(0x56, rpmDir, rpmEsq)
            self.Direcao(angle)           

        except Exception as ex:
            print("Falha ao iniciar o carro: {}".format(ex))

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
        breakingYOLO = False
        #jsonObjectYOLO = ast.literal_eval(msg_objectYOLO.data)
        #breakingYOLO = self.yoloDecision(jsonObjectYOLO)

        self.msgObjectYOLO = breakingYOLO

    def yoloDecision(self, jsonObjectYOLO):
        retorno = False
        msgCanId = 0x00
        param = []
        for object in jsonObjectYOLO:
            if (jsonObjectYOLO[object]['classId'] == "stop sign"):
                print("FREIA PID - Placa PARE")
                retorno = True
                msgCanId = 0x56
                param = [1, 0, 1, 0]                

        return retorno, msgCanId, param
    
    def callbackQRCode(self, msg_QRCode):
        global flagQRCode
        flagQRCode = True
        self.msgQRCode = msg_QRCode


class DecisionMaker():
    def __init__(self):
        # Faz o setup do das veriáveis do carro
        self.rpm_can = 20               #Define a velociade de inicio do carro
        self.angle_can = 25            #Define o angulo de inicio da direção, no ideal começamos com ele ao centro
        self.tSendMsgCAN = time.time() #Inicializa o temporizador de envio de msg na CAN
        self.timeMin = 0.001           #Intervalo de tempo para envio de msg na CAN
    '''
    # sempre enviando as mesagens para o software Grojoba
    def sendInfoSotftware(self, IDcar=3, gap=0, destino=0, localização=0, angCan=25, rpmEsq=0, rpmDir=0):
        msgCanId = 0x94
        param = [IDcar, gap, destino, localização, angCan, rpmEsq, rpmDir]
        vc.sendMsg(self.socket, msgCanId, param)

    # Recebe parametros iniciais do veiculo
    def recInfoSoftware(self):
        msgPlatoon, retornoAng, retornoRPM = vc.logCanPlatoon(self.socket)
        return msgPlatoon, retornoAng, retornoRPM
    '''

async def main_async():
    global flagVehicleCanInit
    global flagQRCode
    global flagObjectYOLOReceived

    rospy.init_node("Decision_maker")
    print("O node Decision Maker foi iniciado")

    nodeDecisionMaker = NodeDecisionMaker()
    dm = DecisionMaker()
    rpmDir = dm.rpm_can
    rpmEsq = dm.rpm_can
    
    flagObjectYOLOReceived = True
    while not flagVehicleCanInit:
        if flagVehiclePositionReceived and flagCurveRadiusReceived and flagDistanceReceived and flagSteeringReceived and flagObjectYOLOReceived:
            flagVehicleCanInit = True 
            # modificar função initCar para receber os parametros da recInfoSoftware e iniciar o veiculo
            nodeDecisionMaker.initCar(rpmDir, rpmEsq, dm.angle_can )
            #dm.sendInfoSotftware(angCan=dm.angle_can, rpmEsq=dm.rpm_can, rpmDir=dm.rpm_can)

    flagQRCode = True
    while not flagQRCode:
        print('Flag QR Code: {}'.format(flagQRCode))
        if flagVehiclePositionReceived and flagCurveRadiusReceived and flagDistanceReceived and flagSteeringReceived:
            flagQRCode = True

    while not rospy.is_shutdown():
        try:
            if dm.timeMin < time.time() - dm.tSendMsgCAN:
                dm.tSendMsgCAN = time.time()
                angDir = nodeDecisionMaker.msgSteering

                '''
                # recebe os parametros inciais definidos no codigo, depois do primiero inicio o software controla os parametros do veiculo
                #msgPlatoon, rpmdir, rpmesq = dm.recInfoSoftware()
                #if msgPlatoon == "":
                rpmDir = dm.rpm_can
                rpmEsq = dm.rpm_can
                #else:
                    #rpmDir = rpmdir
                    #rpmEsq = rpmesq
                '''

                nodeDecisionMaker.Direcao(angDir)
            
                breakingAEB = nodeDecisionMaker.msgDepth
                if not breakingAEB:
                    #breakingYolo, msgCanId, param = nodeDecisionMaker.msgObjectYOLO
                    #if not breakingYolo:
                    #msgCanId = 0x56
                    #param = [1, rpmDir, 1, rpmEsq]
                    #vc.sendMsg(s, msgCanId, param)
                    #print("Acelera: {}; {}".format(msgCanId, param))
                    #else:
                    #    vc.sendMsg(s, msgCanId, param)
                    print("Acelera")
                    nodeDecisionMaker.Powertrain(0x56, rpmDir, rpmEsq)
                else: 
                    #print("AEB: {}; {}".format(msgCanId, param))
                    #vc.sendMsg(s, msgCanId, param)  
                    print("AEB")      
                        

            gc.collect()

        except Exception as ex:
            print("Exception: {}".format(ex))
            pass

        except KeyboardInterrupt:
            print("Exception: KeyboardInterrupt")
            '''
            rpm = 0
            msgCanId = 0x56
            param = [1, rpm, 1, rpm]
            print("FREIA PID - KeyboardInterrupt")
            vc.sendMsg(s, msgCanId, param)

            angDir = 25
            msgCanId = 0x82
            param = [angDir]
            vc.sendMsg(s, msgCanId, param)
            
            s.close()
            '''
            break


def main():
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main_async())


if __name__ == '__main__':
    main()
