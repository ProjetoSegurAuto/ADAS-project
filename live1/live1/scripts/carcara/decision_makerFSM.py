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
import acc
import log

'''
Definição das constantes
'''
RPM_INIT = 50
ANGLE_INIT = 25
STATE_INIT = 0
RPM_ACC_INIT = 0

PLATOON_GAP_INIT = 2

DISTANCE_ACC = 3.0 
DISTANCE_STOP = 1.1 
DISTANCE_BUFFER_LEN = 20
TIME_CAN = 0.01 #0.001 Intervalo de tempo para envio de msg na CAN

class NodeDecisionMaker:
    def __init__(self):
        self.distance_buffer = []
        self.can_params = []
        self.can_id = 0x00

        self.platoon_gap = PLATOON_GAP_INIT
        
        #Flags processamento 
        self.flag_break_aeb = False
        self.flag_break_placa = False
        self.flag_acc = False

        #Flags recebimento de mensagens
        self.flag_depth_received = False
        self.flag_vehicle_position_received = False
        self.flag_steering_received = False
        self.flag_curve_radius_received = False
        self.flag_object_received = False
        self.flag_bridge_received = False
        self.flag_vehicle_can_init = False
        self.flag_qr_code = False

        #Inicializacao de atributos para recebimento de mensagens
        self.msg_depth = Float64()
        self.msg_vehicle_position = Float64()
        self.msg_steering = Float64()
        self.msg_curve_radius = Float64MultiArray()
        self.msg_object = String()
        self.msg_qr_code = Float64()
        self.msg_bridge = list()

        self.orin_message = Int64MultiArray()

        '''
        Subscriber 
        '''
        # Distancias dos objetos detectados na area navegavel
        self.sub_depth = rospy.Subscriber('ObjectsDepth', Float64, self.callback_depth)

        # Posicao do veiculo em relacao ao centro das faixas
        self.sub_vehicle_position = rospy.Subscriber('VehiclePosition', Float64, self.callback_vehicle_position)
        # Angulo de esterçamento do volante
        self.sub_steering = rospy.Subscriber('Steering', Float64, self.callback_steering)
        # Raio de curvatura da pista
        self.sub_curve_radius = rospy.Subscriber('CurveRadius', Float64MultiArray, self.callback_curve_radius)

        # Objetos detectados pela rede neural - YOLO ou Detectnet (trafficcamnet; ssd-mobilenet-v2)
        #self.sub_object_yolo = rospy.Subscriber('ObjectYOLO', String, self.callback_object)
        self.sub_object_yolo = rospy.Subscriber('ObjectDetectnet', String, self.callback_object)

        # QR Code
        self.sub_qr_code = rospy.Subscriber('QRCode', Float64, self.callback_qr_code)

        # CAN Message - Vector e Orin
        self.sub_can_message = rospy.Subscriber('Bridge', Float64MultiArray, self.callback_bridge)
        
        '''
        Publisher 
        '''
        self.pub_decision_maker = rospy.Publisher('Decision_Maker', Int64MultiArray , queue_size=1)

    def callback_depth(self, msg_depth):
        self.flag_depth_received = True
        self.msg_depth = msg_depth.data

        self.distance_buffer.append(self.msg_depth)

        if(len(self.distance_buffer) > DISTANCE_BUFFER_LEN):
            self.distance_buffer.pop(0)

        if(self.msg_depth > DISTANCE_STOP ): #CNN

            distance_valid = [i for i in self.distance_buffer if i < 99 ] 
            
            if(len(distance_valid) > 0):
                distance_valid_mean = sum(distance_valid) / len(distance_valid) 

                if( distance_valid_mean > 0):
                    self.msg_depth = distance_valid_mean
            else:
                self.msg_depth = DISTANCE_ACC + 1  
            
        self.distance_decision()

    def callback_vehicle_position(self, msg_vehicle_position):
        self.flag_vehicle_position_received = True
        self.msg_vehicle_position = msg_vehicle_position.data

    def callback_steering(self, msg_steering):
        self.flag_steering_received = True
        self.msg_steering = int(msg_steering.data)

    def callback_curve_radius(self, msg_radius_curve):
        self.flag_curve_radius_received = True
        self.msg_curve_radius = msg_radius_curve.data

    def callback_object(self, msg_object):
        self.flag_object_received = True
        json_object = ast.literal_eval(msg_object.data)
        self.yolo_decision(json_object)
        self.msg_object = json_object

    def callback_bridge(self, can_message):
        self.flag_bridge_received = True
        self.msg_bridge = list(can_message.data)
        if(len(self.msg_bridge) and hex(int(self.msg_bridge[0])) == '0x95'):
            self.platoon_gap = self.msg_bridge[2]/100

    def callback_qr_code(self, msg_qr_code):
        self.flag_qr_code = True
        self.msg_qr_code = msg_qr_code

    def pubOrinToInfra(self):
        self.orin_message.data =  self.can_params 
        self.pub_decision_maker.publish(self.orin_message)

    def distance_decision(self):
        distance = self.msg_depth
        print("Distancia {}".format(distance))

        self.flag_break_aeb = False
        self.flag_acc = False
        
        if not np.isnan(distance):

            if np.isfinite(distance):

                if distance < DISTANCE_STOP:
                    #print("TRAVA RODA - FREIA PWM - Distância: {}".format(distance))
                    self.flag_break_aeb = True

                elif distance < DISTANCE_ACC:
                    self.flag_acc = True
                    #print("ACC - Distância: {} | Flag ACC: {}".format(distance, flag_acc))
                   
            else:
                #print("TRAVA RODA ELSE- FREIA PWM - Distância: {}".format(distance))
                self.flag_break_aeb = True

        else:
            print("NAN | distancia {}".format(distance))
            

    def yolo_decision(self, json_object):
        self.flag_break_placa = False
        for object in json_object:
            if json_object[object]['classId'] == "stop sign":
                #print("FREIA PID - Placa PARE")
                self.flag_break_placa = True
                self.can_id = 0x56
                self.can_params = [1, 0, 1, 0, self.can_id]

class DecisionMakerFSM:
    
    def __init__(self, node_decision_maker):

        ndm = node_decision_maker

        self.flag_vehicle_can_init = False
        self.dic_states = {0: 'INITIAL', 1: 'NORMAL_DRIVING', 2: 'EMERGENCY_BRAKE', 3:'ACC'}
        self.state_transitions = {
            0: lambda: 1 if self.flag_vehicle_can_init else 0,
            1: lambda: 2 if ndm.flag_break_aeb or ndm.flag_break_placa else ( 3 if ndm.flag_acc else 1 ),
            2: lambda: 1 if not ndm.flag_break_aeb and not ndm.flag_break_placa and not ndm.flag_acc else ( 3 if not ndm.flag_break_aeb and not ndm.flag_break_placa and ndm.flag_acc else 2),
            3: lambda: 2 if ndm.flag_break_aeb or ndm.flag_break_placa else (1 if not ndm.flag_acc else 3)
        }
        
        self.state = STATE_INIT
        self.rpm_init = RPM_INIT 
        self.rpm_can =  0
        self.angle_can = ANGLE_INIT
        self.time_min = TIME_CAN
        self.gap = ndm.platoon_gap
        
        # Inicializa o temporizador de envio de msg na CAN
        self.t_send_msg_can = time.time() 
        # Inicializa os buffers do ACC com valores iguais
        self.ACC_bufferError = [ndm.msg_depth, ndm.msg_depth]
        self.ACC_bufferTime = [self.t_send_msg_can, self.t_send_msg_can]
       
        self.acc = acc.controllerFuzzy() 

    def update_state(self, node_decision_maker):
        self.state = self.state_transitions[self.state]()
        self.actions(node_decision_maker)

    def actions(self, node_decision_maker):
        ndm = node_decision_maker
        self.gap = ndm.platoon_gap

        if self.state == 0:
            if ndm.flag_vehicle_position_received and ndm.flag_steering_received and ndm.flag_curve_radius_received and ndm.flag_depth_received and ndm.flag_object_received:
                print("Estado: {}. Inicialização do carro autorizada!".format(self.dic_states[self.state]))
                self.flag_vehicle_can_init = True
                try:
                    self.rpm_can =  self.rpm_init
                    ndm.can_id = 0x56
                    ndm.can_params = [1, self.rpm_can, 1, self.rpm_can, ndm.can_id]
                    ndm.pubOrinToInfra()
                    #print("Mensagem para ECU POWERTRAIN: ", msg_can_id, param)

                    ndm.can_id = 0x82
                    ndm.can_params = [self.angle_can, ndm.can_id]
                    ndm.pubOrinToInfra()
                    #print("Mensagem para ECU DIREÇÃO: ", msg_can_id, param)

                except Exception as ex:
                    print("Falha ao iniciar o carro: {}".format(ex))

        elif self.state == 1:                
            print("Estado: {}. Seguindo as faixas".format(self.dic_states[self.state]))
            if self.time_min < time.time() - self.t_send_msg_can:
                self.t_send_msg_can = time.time()

                ndm.can_id = 0x82
                ndm.can_params = [ndm.msg_steering, ndm.can_id]
                ndm.pubOrinToInfra()

                self.rpm_can =  self.rpm_init
                ndm.can_id = 0x56
                ndm.can_params = [1, self.rpm_can, 1, self.rpm_can, ndm.can_id]
                ndm.pubOrinToInfra()

        elif self.state == 2:
            print("Estado: {}. Emergência!".format(self.dic_states[self.state]))
            if self.time_min < time.time() - self.t_send_msg_can:
                self.t_send_msg_can = time.time()
                self.rpm_can  = 0

                ndm.can_id = 0x56
                ndm.can_params = [1, self.rpm_can , 1, self.rpm_can , ndm.can_id]
                ndm.pubOrinToInfra()
                time.sleep(0.2)
                ndm.can_id = 0x5C
                ndm.can_params = [1, self.rpm_can , 1, self.rpm_can , ndm.can_id]
                ndm.pubOrinToInfra()
        
        elif self.state == 3:  
            print("Estado: {}.".format(self.dic_states[self.state]))
            if self.time_min < time.time() - self.t_send_msg_can:
                
                self.ACC_bufferTime[0] = self.t_send_msg_can
                self.ACC_bufferTime[1] = time.time()
                self.t_send_msg_can = time.time()

                ndm.can_id = 0x82
                ndm.can_params = [ndm.msg_steering, ndm.can_id]
                ndm.pubOrinToInfra()
                
                can_msg = list()
                can_msg = ndm.msg_bridge
                
                if(len(can_msg) and hex(int(can_msg[0])) == '0x50'):
                    rpm_left = can_msg[3]
                    rpm_right = can_msg[5]
                    
                    if(abs(rpm_left-rpm_right) < 5):

                        rpm_mean = int((rpm_left + rpm_right)/2)
                        #print("RPM MEAN: {}".format(rpm_mean))
                        print("node_decision_maker.msg_depth: {}".format(ndm.msg_depth))
                        erro = (ndm.msg_depth - self.gap)*100
                        self.ACC_bufferError[0] = self.ACC_bufferError[1]
                        self.ACC_bufferError[1] = erro
                        dErro = (self.ACC_bufferError[1] - self.ACC_bufferError[0])/(self.ACC_bufferTime[1] - self.ACC_bufferTime[0])
                        rpm_acc = int(self.acc.controller(erro, dErro))
                        self.rpm_can = rpm_mean + rpm_acc

                        if self.rpm_can > (RPM_INIT + 5):
                            self.rpm_can = (RPM_INIT + 5)
                        elif self.rpm_can < 25:
                            self.rpm_can = 25
                            
                        ndm.can_id = 0x56
                        ndm.can_params = [1, self.rpm_can, 1, self.rpm_can, ndm.can_id]
                        ndm.pubOrinToInfra()

                        linha_arquivo =[[time.time(), ndm.msg_depth, erro, dErro, rpm_acc, rpm_mean, self.rpm_can]]
                        columns = ['time', 'distance', 'error', 'dError', 'out_acc(rpm)', 'rpm_mean' ,'rpm_can']
                        #print(linha_arquivo)
                        log.save_dataframe_to_csv(linha_arquivo, columns)
                    
                else:
                    del can_msg[:]
        
        print("Velocidade {} RPM".format(self.rpm_can))
                
        
def main():
    rospy.init_node("Decisionmaker")
    print("O node Decision Maker foi iniciado")

    node_decision_maker = NodeDecisionMaker()
    decision_maker_fsm = DecisionMakerFSM(node_decision_maker)

    while not rospy.is_shutdown():
        try:
            decision_maker_fsm.update_state(node_decision_maker)
            gc.collect()

        except Exception as ex:
            print("Exception: {}".format(ex))

        except KeyboardInterrupt:
            print("Exception: KeyboardInterrupt")

            rpm = 0
            node_decision_maker.can_id = 0x56
            node_decision_maker.can_params = [1, rpm, 1, rpm, node_decision_maker.can_id ]
            #print("FREIA PID - KeyboardInterrupt")
            node_decision_maker.pubOrinToInfra()

            ang_dir = 25
            node_decision_maker.can_id = 0x82
            node_decision_maker.can_params = [ang_dir, node_decision_maker.can_id]
            node_decision_maker.pubOrinToInfra()

            break


if __name__ == '__main__':
    main()


#      #   #         #   ####
#      #    #       #    #
#      #     #     #     ####
#      #      #   #      #
####   #        #        ####
