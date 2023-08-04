#!/usr/bin/env python3

##########
# Informações sobre esse Node:
# Nome: Decision maker
# Descrição: A partir das informações toma a decisão

import rospy
import time
import vector as vc
import numpy as np
import asyncio
from threading import Thread
from std_msgs.msg import Float64, Float64MultiArray, String, Int32
import ast
import gc

flagDistanceReceived = False
flagVehiclePositionReceived = False
flagSteeringReceived = False
flagCurveRadiusReceived = False
flagObjectYOLOReceived = False
flagVehicleCanInit = False
flagQRCode = False

flagThrottle = True
flagBreakAEB = False
flagBreakYOLO = False
flagBreakMSGLeader = False #implementar em algum callback (mensagem platoon)
canID = 0x00
canParams = [] 

flagReceiveLeader = False

ID_CAR = 0

class NodeDecisionMaker():
    def __init__(self):
        self.msgDepth = Float64()
        self.msgVehiclePosition = Float64()
        self.msgSteering = Float64()
        self.msgCurveRadius = Float64MultiArray()
        self.msgObjectYOLO = String()
        self.msgQRCode = Float64()
        self.msgMyLeader = Int32()

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
        self.subModeling = rospy.Subscriber(
            'TPC9Leader', Int32, self.callbackMyLeader)

    def callbackDepth(self, msg_depth):
        global flagDistanceReceived

        flagDistanceReceived = True
        self.AEB(msg_depth.data)

        self.msgDepth = msg_depth.data

    def AEB(self, distance):
        global flagBreakAEB
        global flagBreakYOLO
        global canID
        global canParams

        flagBreakAEB = False
        distanceBreak = 1.4#1.3 #1.2      #Define a distância que o freio de emergência será acionado
        distanceStop  = 1.0#0.9 #0.5      #Define a distância que o carro vai parar

        if not np.isnan(distance):
            if np.isfinite(distance):
                if distance <= distanceStop:
                    canID = 0x5C
                    flagBreakAEB = True
                elif distance <= distanceBreak and not flagBreakYOLO:
                    canID = 0x56
                    flagBreakAEB = True
            else:
                canID = 0x5C
                flagBreakAEB = True

        if flagBreakAEB:     
            canParams = [1, 0, 1, 0]

 
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
        jsonObjectYOLO = ast.literal_eval(msg_objectYOLO.data)
        self.yoloDecision(jsonObjectYOLO)

        self.msgObjectYOLO = jsonObjectYOLO

    def yoloDecision(self, jsonObjectYOLO):
        global flagBreakYOLO
        global flagBreakAEB
        global canID
        global canParams

        #flagBreakYOLO = False
        if(not flagBreakAEB):
            for object in jsonObjectYOLO:
                if (jsonObjectYOLO[object]['classId'] == "stop sign"):
                    print("FREIA PID - Placa PARE")
                    flagBreakYOLO = True
                    canID = 0x56
                    canParams = [1, 0, 1, 0]                
    
    def callbackQRCode(self, msg_QRCode):
        global flagQRCode
        flagQRCode = True
        self.msgQRCode = msg_QRCode

    def callbackMyLeader(self, msg_leader):
        global flagReceiveLeader
        flagReceiveLeader = True
        self.msgMyLeader = msg_leader.data

class DecisionMaker():
    def __init__(self):
        # Faz o setup do das veriáveis do carro
        self.rpm_can = 60              #Define a velociade de inicio do carro
        self.angle_can = 25            #Define o angulo de inicio da direção, no ideal começamos com ele ao centro
        self.tSendMsgCAN = time.time() #Inicializa o temporizador de envio de msg na CAN
        self.timeMin = 0.001           #Intervalo de tempo para envio de msg na CAN
        self.socket = vc.openSocket()  #Inicializa o socket de comunicação com a CAN

    def initCar(self):
        print("Inicialização do carro autorizada!")
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
            print("Falha ao iniciar o carro: {}".format(ex))

    # sempre enviando as mesagens para o software Grojoba
    def sendInfoSotftware(self, IDcar=3, gap=0, destino=0, localização=0, angCan=25, rpmEsq=0, rpmDir=0):
        msgCanId = 0x94
        param = [IDcar, gap, destino, localização, angCan, rpmEsq, rpmDir]
        vc.sendMsg(self.socket, msgCanId, param)

    # Recebe parametros iniciais do veiculo
    def recInfoSoftware(self):
        msgPlatoon, retornoAng, retornoRPM = vc.logCanPlatoon(self.socket)
        return msgPlatoon, retornoAng, retornoRPM

async def main_async():
    global flagVehicleCanInit
    global flagQRCode
    global flagObjectYOLOReceived

    global flagBreakAEB
    global flagBreakYOLO
    global canID
    global canParams
    global flagThrottle

    global ID_CAR
    global MY_LEADER

    rospy.init_node("Decision_maker")
    print("O node Decision Maker foi iniciado")

    nodeDecisionMaker = NodeDecisionMaker()
    dm = DecisionMaker()
    s = dm.socket
    rpmDir = dm.rpm_can
    rpmEsq = dm.rpm_can

    flagBreak = False
    
    while not flagVehicleCanInit:
        if flagVehiclePositionReceived and flagCurveRadiusReceived and flagDistanceReceived and flagSteeringReceived and flagObjectYOLOReceived:
            flagVehicleCanInit = True 
            dm.initCar()

    while not rospy.is_shutdown():
        try:
            if dm.timeMin < time.time() - dm.tSendMsgCAN:
                dm.tSendMsgCAN = time.time()
                print(f"my id: {ID_CAR} | my leader: {nodeDecisionMaker.msgMyLeader}")
                if(ID_CAR == nodeDecisionMaker.msgMyLeader):
                    angDir = nodeDecisionMaker.msgSteering
                    msgCanId = 0x82
                    param = [angDir]
                    vc.sendMsg(s, msgCanId, param)

                    if (flagBreakAEB or flagBreakYOLO):
                        vc.sendMsg(s, canID, canParams)
                        print("Pare: {}; {}".format(hex(canID), canParams))
                        print("flagBreakAEB: {}; flagBreakYOLO: {}; flagThrottle: {}.".format(flagBreakAEB, flagBreakYOLO, flagThrottle))

                        flagThrottle = True 

                    elif flagThrottle:
                            canID = 0x56
                            canParams = [1, rpmDir, 1, rpmEsq]
                            vc.sendMsg(s, canID, canParams)
                            print("Acelera: {}; {}".format(hex(canID), canParams))  
                            print("flagBreakAEB: {}; flagBreakYOLO: {}; flagThrottle: {}.".format(flagBreakAEB, flagBreakYOLO, flagThrottle))
                            
                            flagThrottle = False 
                else:
                    #comportamento do filho
                    angDir = None # Recece do callback do Aruco
                    msgCanId = 0x82
                    param = [angDir]
                    vc.sendMsg(s, msgCanId, param)

                    if (flagBreakMSGLeader or flagBreakAEB or flagBreakYOLO):
                        canID = 0x56
                        canParams = [1, 0, 1, 0]
                        vc.sendMsg(s, canID, canParams)
                        print("Pare: {}; {}".format(hex(canID), canParams))

                        flagThrottle = True 

                    elif flagThrottle: # ta faltando um atraso no eixo X 
                        canID = 0x56
                        canParams = [1, rpmDir, 1, rpmEsq]
                        vc.sendMsg(s, canID, canParams)
                        print("Acelera: {}; {}".format(hex(canID), canParams))  
                        print("flagBreakAEB: {}; flagBreakYOLO: {}; flagThrottle: {}.".format(flagBreakAEB, flagBreakYOLO, flagThrottle))
                        
                        flagThrottle = False 
                

            print(vc.logCAN(s))
            gc.collect()

        except Exception as ex:
            print("Exception: {}".format(ex))
            pass

        except KeyboardInterrupt:
            print("Exception: KeyboardInterrupt")
 
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
            break


def main():
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main_async())


if __name__ == '__main__':
    main()
