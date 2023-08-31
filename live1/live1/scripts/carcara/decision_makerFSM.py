#!/usr/bin/env python3

##########
# Informações sobre esse Node:
# Nome: Decision maker
# Descrição: A partir das infromações toma a decisão

import rospy
import time
import numpy as np
from threading import Thread
from std_msgs.msg import Float64, Float64MultiArray, Int64MultiArray, String, Int32
import ast
import gc

flag_distance_received = False
flag_vehicle_position_received = False
flag_steering_received = False
flag_curve_radius_received = False
flag_object_yolo_received = False
flag_vehicle_can_init = False
flag_qr_code = False

flag_throttle = True
flag_break_aeb = False
flag_break_yolo = False
can_id = 0x00
can_params = []

MY_ID = 0
GAP = 10

class NodeDecisionMaker:
    def __init__(self):
        self.msg_depth = Float64()
        self.msg_vehicle_position = Float64()
        self.msg_steering = Float64()
        self.msg_curve_radius = Float64MultiArray()
        self.msg_object_yolo = String()
        self.msg_qr_code = Float64()
        self.__can_message = list()
        self.__flag_receive_can_msg = False
        self.__myLeader = MY_ID

        self.sub_depth = rospy.Subscriber('TPC3Depth', Float64, self.callback_depth)
        self.sub_vehicle_position = rospy.Subscriber('TPC4VehiclePosition', Float64, self.callback_vehicle_position)
        self.sub_steering = rospy.Subscriber('TPC4Steering', Float64, self.callback_steering)
        self.sub_curve_radius = rospy.Subscriber('TPC5CurveRadius', Float64MultiArray, self.callback_curve_radius)
        self.sub_object_yolo = rospy.Subscriber('TPC3ObjectYOLO', String, self.callback_object_yolo)
        self.sub_qr_code = rospy.Subscriber('TPC6QRCode', Float64, self.callback_qr_code)
        self.sub_can_message = rospy.Subscriber('TPC10Bridge', Float64MultiArray, self.callback_logger)
        self.subModeling = rospy.Subscriber('TPC9Leader', Int32, self.callbackMyLeader)

        self.pubData = rospy.Publisher('TPC10Decision_Maker', Int64MultiArray , queue_size=1)

    def callback_depth(self, msg_depth):
        global flag_distance_received

        flag_distance_received = True
        self.aeb(msg_depth.data)

        self.msg_depth = msg_depth.data

    def callback_vehicle_position(self, msg_vehicle_position):
        global flag_vehicle_position_received
        flag_vehicle_position_received = True
        self.msg_vehicle_position = msg_vehicle_position.data

    def callback_steering(self, msg_steering):
        global flag_steering_received
        flag_steering_received = True
        self.msg_steering = int(msg_steering.data)

    def callback_curve_radius(self, msg_radius_curve):
        global flag_curve_radius_received
        flag_curve_radius_received = True
        self.msg_curve_radius = msg_radius_curve.data

    def callback_object_yolo(self, msg_object_yolo):
        global flag_object_yolo_received

        flag_object_yolo_received = True
        json_object_yolo = ast.literal_eval(msg_object_yolo.data)
        self.yolo_decision(json_object_yolo)

        self.msg_object_yolo = json_object_yolo

    def callback_logger(self, can_message):
        self.__flag_receive_can_msg = True
        self.__can_message = list(can_message.data)
        print("AQUIII: ",end='')
        print(self.__can_message)
    
    def getCANMessage(self):
        return self.__can_message

    def getFlagLogger(self) -> bool:
        return self.__flag_receive_can_msg

    def setFlagLogger(self, value: bool):
        self.__flag_receive_can_msg = value

    def callbackMyLeader(self, msg_leader):
        self.__myLeader = int(msg_leader.data)

    def getWhatIsMyLeader(self):
        return self.__myLeader

    def pubOrinToInfra(self, orin_message: list):
        print("pubOrinToInfra")
        self.__orin_message = Int64MultiArray()
        self.__orin_message.data = orin_message 
        self.pubData.publish(self.__orin_message)

    def yolo_decision(self, json_object_yolo):
        global flag_break_yolo
        global can_id
        global can_params

        flag_break_yolo = False
        for object in json_object_yolo:
            if json_object_yolo[object]['classId'] == "stop sign":
                print("FREIA PID - Placa PARE")
                flag_break_yolo = True
                can_id = 0x56
                can_params = [1, 0, 1, 0]

    def aeb(self, distance):
        global flag_break_aeb
        global can_id
        global can_params

        flag_break_aeb = False
        distance_break = 1.4
        distance_stop = 1.0

        if not np.isnan(distance):
            if np.isfinite(distance):
                if distance < distance_stop:
                    print("TRAVA RODA - FREIA PWM - Distância: {}".format(distance))
                    can_id = 0x5C
                    flag_break_aeb = True
                elif distance < distance_break:
                    print("FREIA PID - Distância: {}".format(distance))
                    can_id = 0x56
                    flag_break_aeb = True
            else:
                print("TRAVA RODA - FREIA PWM - Distância: {}".format(distance))
                can_id = 0x5C
                flag_break_aeb = True

        if flag_break_aeb:
            can_params = [1, 0, 1, 0]

    def callback_qr_code(self, msg_qr_code):
        global flag_qr_code
        flag_qr_code = True
        self.msg_qr_code = msg_qr_code


class DecisionMakerFSM:
    global flag_vehicle_can_init
    global flag_break_aeb
    global flag_break_yolo
    global can_id
    global can_params

    def __init__(self, node_decision_maker):
        self.dic_states = {0: 'INITIAL',1: 'NORMAL_DRIVING', 2: 'EMERGENCY_BRAKE'}

        self.state_transitions = {
            0: lambda: 1 if flag_vehicle_can_init else 0,
            1: lambda: 2 if flag_break_aeb or flag_break_yolo else 1,
            2: lambda: 1 if  not flag_break_aeb and not flag_break_yolo and node_decision_maker.msg_depth > self.safe_distance else 2
        }   
        self.safe_distance = 1.4  #alterar para que a distacia de parada seja uma flag global  

        self.state = 0
        self.rpm_can = 40  # Define a velociade de inicio do carro
        # Define o angulo de inicio da direção, no ideal começamos com ele ao centro
        self.angle_can = 25
        # Inicializa o temporizador de envio de msg na CAN
        self.t_send_msg_can = time.time()
        self.time_min = 0.001  # Intervalo de tempo para envio de msg na CAN
        self.emergency_break_start = 0
        self.wait_time = 1 #em segundos

    def update_state(self, node_decision_maker):
        self.state = self.state_transitions[self.state]()
        self.actions(node_decision_maker)

    def actions(self, node_decision_maker):
        global flag_vehicle_can_init
        if self.state == 0:
            if flag_vehicle_position_received and flag_steering_received and flag_curve_radius_received and flag_distance_received and flag_object_yolo_received:
                print("Estado: {}. Inicialização do carro autorizada!".format(self.dic_states[self.state]))
                flag_vehicle_can_init = True
                try:
                    msg_can_id = 0x56
                    param = [1, self.rpm_can, 1, self.rpm_can, msg_can_id]
                    node_decision_maker.pubOrinToInfra(param)
                    print("Mensagem para ECU POWERTRAIN: ", msg_can_id, param)

                    msg_can_id = 0x82
                    param = [self.angle_can, msg_can_id]
                    node_decision_maker.pubOrinToInfra(param)
                    print("Mensagem para ECU DIREÇÃO: ", msg_can_id, param)

                except Exception as ex:
                    print("Falha ao iniciar o carro: {}".format(ex))

        elif self.state == 1:
            print("Estado: {}. Seguindo as faixas".format(self.dic_states[self.state]))
            if self.time_min < time.time() - self.t_send_msg_can:
                self.t_send_msg_can = time.time()
                ang_dir = node_decision_maker.msg_steering

                msg_can_id = 0x82
                param = [ang_dir, msg_can_id]
                node_decision_maker.pubOrinToInfra(param)

                msg_can_id = 0x56
                param = [1, self.rpm_can, 1, self.rpm_can, msg_can_id]
                node_decision_maker.pubOrinToInfra(param)

                destiny = 13
                localization = 7
                param = [MY_ID, GAP, destiny, localization, ang_dir, self.rpm_can, self.rpm_can, 0x94]
                node_decision_maker.pubOrinToInfra(param)

        elif self.state == 2:
            print("Estado: {}. Emergência!".format(self.dic_states[self.state]))
            if self.time_min < time.time() - self.t_send_msg_can:
                self.t_send_msg_can = time.time()
                param = can_params 
                param.append(can_id)
                node_decision_maker.pubOrinToInfra(param)

                destiny = 13
                localization = 7
                param = [MY_ID, GAP, destiny, localization, node_decision_maker.msg_steering, 0, 0, 0x94]
                node_decision_maker.pubOrinToInfra(param)       


def main():
    rospy.init_node("Decisionmaker")
    print("O node Decision Maker foi iniciado")

    node_decision_maker = NodeDecisionMaker()

    decision_maker_fsm = DecisionMakerFSM(node_decision_maker)

    while not rospy.is_shutdown():
        try:
            if(MY_ID == node_decision_maker.getWhatIsMyLeader()): #Se o carro é lider
                decision_maker_fsm.update_state(node_decision_maker)
                decision_maker_fsm.actions(node_decision_maker)
            else:# Se o carro é filho
                #leitura de dados da CAN(dados do Pai)
                can_msg = node_decision_maker.getCANMessage()
                leader = int()
                gap = float()
                destiny = int()
                localization = int()
                direction = float()
                rpm_left = int()
                rpm_right = int()

                if(can_msg[0] == 1):
                    leader = can_msg[1]
                    gap = can_msg[3]
                    destiny = can_msg[4]
                    localization = can_msg[5]
                    direction = can_msg[6]
                    rpm_left = can_msg[7]
                    rpm_right = can_msg[8]
                
                #implemente nesta linha a FSM para o filho!

            gc.collect()

        except Exception as ex:
            print("Exception: {}".format(ex))

        except KeyboardInterrupt:
            print("Exception: KeyboardInterrupt")

            rpm = 0
            msg_can_id = 0x56
            param = [1, rpm, 1, rpm, msg_can_id]
            print("FREIA PID - KeyboardInterrupt")
            node_decision_maker.pubOrinToInfra(param)

            ang_dir = 25
            msg_can_id = 0x82
            param = [ang_dir, msg_can_id]
            node_decision_maker.pubOrinToInfra(param)

            break


if __name__ == '__main__':
    main()


#      #   #         #   ####
#      #    #       #    #
#      #     #     #     ####
#      #      #   #      #
####   #        #        ####
