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

flagThrottle = True
flagBreakAEB = False
flagBreakYOLO = False
canID = 0x00
canParams = []


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
        self.AEB(msg_depth.data)

        self.msgDepth = msg_depth.data

    def AEB(self, distance):
        global flagBreakAEB
        global canID
        global canParams

        flagBreakAEB = False
        distanceBreak = 1.8  # 1.2      #Define a distância que o freio de emergência será acionado
        distanceStop = 1.0  # 0.5      #Define a distância que o carro vai parar

        if not np.isnan(distance):
            if np.isfinite(distance):
                if distance < distanceStop:
                    # print("TRAVA RODA - FREIA PWM - Distância: {}".format(distance))
                    canID = 0x5C
                    flagBreakAEB = True
                elif distance < distanceBreak:
                    # print("FREIA PID - Distância: {}".format(distance))
                    canID = 0x56
                    flagBreakAEB = True
            else:
                # print("TRAVA RODA - FREIA PWM - Distância: {}".format(distance))
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
        global canID
        global canParams

        flagBreakYOLO = False
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


class DecisionMakerFSM():
    global flagVehicleCanInit
    global flagBreakAEB
    global flagBreakYOLO
    global canID
    global canParams

    def __init__(self):
        self.state = "INITIAL"
        self.rpm_can = 50  # Define a velociade de inicio do carro
        # Define o angulo de inicio da direção, no ideal começamos com ele ao centro
        self.angle_can = 25
        self.tSendMsgCAN = time.time()  # Inicializa o temporizador de envio de msg na CAN
        self.timeMin = 0.001  # Intervalo de tempo para envio de msg na CAN
        self.socket = vc.openSocket()  # Inicializa o socket de comunicação com a CAN

    def update_state(self, node_decision_maker):
        if self.state == "INITIAL":
            if flagVehicleCanInit:
                self.state = "NORMAL_DRIVING"
        elif self.state == "NORMAL_DRIVING":
            if flagBreakAEB or flagBreakYOLO:
                self.state = "EMERGENCY_BRAKE"
        elif self.state == "EMERGENCY_BRAKE":
            if not flagBreakAEB and not flagBreakYOLO:
                self.state = "NORMAL_DRIVING"

        self.actions(node_decision_maker)

    def actions(self, node_decision_maker):

        if self.state == "INITIAL":
            print("Inicialização do carro autorizada!")
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
                print("Falha ao iniciar o carro: {}".format(ex))
                pass

        elif self.state == "NORMAL_DRIVING":
            print("Seguindo as faixas")
            if self.timeMin < time.time() - self.tSendMsgCAN:
                self.tSendMsgCAN = time.time()
                angDir = NodeDecisionMaker().msgSteering

                msgCanId = 0x82
                param = [angDir]
                vc.sendMsg(self.socket, msgCanId, param)

        elif self.state == "EMERGENCY_BRAKE":
            vc.sendMsg(self.socket, canID, canParams)
            print("Pare: {}; {}".format(hex(canID), canParams))
            print("flagBreakAEB: {}; flagBreakYOLO: {}.".format(
                flagBreakAEB, flagBreakYOLO))


def main():
    loop = asyncio.get_event_loop()
    decision_maker_fsm = DecisionMakerFSM()
    node_decision_maker = NodeDecisionMaker()

    s = decision_maker_fsm.socket

    while not rospy.is_shutdown():
        try:
            decision_maker_fsm.update_state(node_decision_maker)
            decision_maker_fsm.actions(node_decision_maker)

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


if __name__ == '__main__':
    main()
