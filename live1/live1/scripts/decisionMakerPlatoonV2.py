#!/usr/bin/env python3

##########
#Informações sobre esse Node:
#Nome: Decision maker
#Descrição: A partir das infromações toma a decisão

import rospy
import math
import time
import vector as vc                 #biblioteca dda vector que faz a comunicação da orin com a Rede CAN
import numpy as np
import socket
from threading import Thread
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray, String
import ast
import gc

HOST = "192.168.1.101"  # Standard loopback interface address (localhost)
PORT = 2323  # Port to listen on (non-privileged ports are > 1023)

flagDistanceReceived = False
flagVehiclePositionReceived = False
flagSteeringReceived = False
flagCurveRadiusReceived = False
flagObjectYOLOReceived = False
flagVehicleCanInit = False
flagQRCode = False

class NodeDecisionMaker():

    def __init__(self):
        self.msgDepth = Float64()           #recebe a mensagem do node depth
        self.msgVehiclePosition = Float64()         #recebe a mensagem do posição d veiculo
        self.msgSteering = Float64()         #recebe a mensagem do angulo do volante
        self.msgCurveRadius = Float64MultiArray()         #recebe a mensagem do posição raio de curvatura
        self.msgObjectYOLO = String()
        self.left_curverad = None
        self.right_curverad = None
        self.msgQRCode = Float64()

        self.subDepth = rospy.Subscriber('TPC3Depth', Float64, self.callbackDepth)
        self.subVehiclePosition = rospy.Subscriber('TPC4VehiclePosition', Float64, self.callbackVehiclePosition)
        self.subSteering = rospy.Subscriber('TPC4Steering', Float64, self.callbackSteering)
        self.subCurveRadius = rospy.Subscriber('TPC5CurveRadius', Float64MultiArray, self.callbackCurveRadius)
        self.subObjectYOLO = rospy.Subscriber('TPC3ObjectYOLO', String, self.callbackObjectYOLO)
        self.subQRCode = rospy.Subscriber('TPC6QRCode',Float64,self.callbackQRCode)
    
    #callback para receber a mensagem do node depth
    def callbackDepth(self,msg_depth):
        global flagDistanceReceived
        flagDistanceReceived = True
        self.msg_depth = msg_depth.data          #passa a mensagem recebida para o atributo da classe
        #self.msg_depth = float(msg_depth)
        #print("distancia_decision_Maker: ",msg_depth," m")
    
    def callbackVehiclePosition(self,msg_vehiclePosition):
        global flagVehiclePositionReceived
        flagVehiclePositionReceived = True
        self.msgVehiclePosition = msg_vehiclePosition.data
        #self.msgVehiclePosition = float(msg_vehiclePosition)
        #print("vehicle_position_decision_Maker: ",msg_vehiclePosition," m")
        #print(type(msg_vehiclePosition))
    
    def callbackSteering(self, msg_steering):
        global flagSteeringReceived
        flagSteeringReceived = True
        self.msgSteering = int(msg_steering.data)

    #callback para receber o raio de curvatura
    def callbackCurveRadius(self,msg_radiusCurve):
        global flagCurveRadiusReceived 
        flagCurveRadiusReceived = True
        self.msgCurveRadius = msg_radiusCurve.data
        self.left_curverad = self.msgCurveRadius[0]
        self.right_curverad = self.msgCurveRadius[1]
        #self.left_curverad = float(self.msgCurveRadius[0])
        #self.right_curverad = float(self.msgCurveRadius[1])
        #print("left_curverad ",self.left_curverad," m")
        #print("right_curverad ",self.right_curverad," m")

    def callbackObjectYOLO(self, msg_objectYOLO):
        global flagObjectYOLOReceived 
        flagObjectYOLOReceived = True
        self.msgObjectYOLO = msg_objectYOLO
    
    def callbackQRCode(self, msg_QRCode):
        global flagQRCode
        flagQRCode = True
        self.msgQRCode = msg_QRCode
        print(self.msgQRCode.data)
    
#classe do NodeDecisionMaker:
class DecisionMaker():

    def __init__(self):
        """Faz o setup do das veriáveis do carro"""
        self.rpm_can = 0                #Define a velociade de inicio do carro
        self.angle_can = 25                 #Define o angulo de inicio da direção, no ideal começamos com ele ao centro
        self.distanceBreak = 1.2            #Define a distân que o freio de emergência será acionado
        self.distanceStop = 0.5             #Define a distân que o carro vai parar
        self.tSendMsgCAN = time.time()      #não sei...
        self.angMin = 1
        self.angMax = 50
        self.timeMin = 0.005
        self.socket = vc.openSocket()

    #inicia o carro
    def initCar(self):
        """Inicia o carro"""
        print("iniciando carro autorizado!")
        try:
            #enviar mensagens para ECU do powertrain
            msgCanId = 0x56
            #[Direcao Esq, RPM Esq, #Direcao Dir, RPM Dir]
            param = [1, self.rpm_can, 1, self.rpm_can]          #Preenchendo os parametros para o envio da mensagem
            vc.sendMsg(self.socket, msgCanId, param)
            print("mensagem para POWERTRAIN: ",msgCanId,param)
    
            #enviar mensagens para ECU de direção
            msgCanId = 0x82
            param = [self.angle_can]                            #Preenchendo os parametros para o envio da mensagem
            vc.sendMsg(self.socket, msgCanId, param)
            print("mensagem parar Direção: ",msgCanId,param)

        except Exception as ex:
            print("Exception initCar: {}".format(ex))
    
    #Automatic emergency braking (AEB)
    def AEB(self,distance): 
        retorno = False
        msgCanId = 0x00
        try:
            if not np.isnan(distance):
                if np.isfinite(distance):
                    if distance < self.distanceStop:
                        print("STOP!!! - distance: {}".format(distance))               
                        #TRAVA RODA - FREIA PWM
                        msgCanId = 0x5C
                        retorno = True

                    elif distance < self.distanceBreak:
                        print("Break!!! - distance: {}".format(distance))                    
                        #FREIA PID
                        msgCanId = 0x56
                        retorno = True
                else:
                    print("STOP!!! infinite - distance: {}".format(distance))                   
                    #TRAVA RODA - FREIA PWM
                    msgCanId = 0x5C
                    retorno = True
                    
                #[Direcao Esq, PWM Esq, #Direcao Dir, PWM Dir]
                param = [1, 0, 1, 0]
                #comentar essa linha para testar sem a vector
                vc.sendMsg(self.socket, msgCanId, param)

                return retorno

        except Exception as ex:
            print("Exception: {}".format(ex)) 
            return retorno

def main():

    global flagVehicleCanInit       #Para poder usar a Flag que autoriza o inicio do carro
    global flagQRCode
    global flagObjectYOLOReceived 


    rospy.init_node("Decision_maker")           #inicia o no ROS
    print("O node Decision maker foi iniciado")

    nodeDecisionMaker = NodeDecisionMaker()         #instancia o node ROS
    dm = DecisionMaker()                 #Instancia o objeto LKA

    #loop para aguardar o recebimentos das mensagens e autorizar o inicio do carro.
    while(not flagVehicleCanInit):
        if(flagVehiclePositionReceived & flagCurveRadiusReceived & flagDistanceReceived & flagSteeringReceived):           #Se o node recebeu todas as mensagens ele pode seguir
            flagVehicleCanInit=True
            dm.initCar()

    while(not flagQRCode):
        print('Flag qr code:{}'.format(flagQRCode))
        if(flagVehiclePositionReceived & flagCurveRadiusReceived & flagDistanceReceived & flagSteeringReceived):           #Se o node recebeu todas as mensagens ele pode seguir
            flagQRCode=True

    #Loop de trabalho, tendo passado pelo loop de inicio podemos começar a tratar os dados:
    s = dm.socket
    print(s)
    #logThread.start()
    while(not rospy.is_shutdown() and flagQRCode):
        try:

            print("QR Code detectado pode iniciar: ", nodeDecisionMaker.msgQRCode.data)
            print('qr code iderntificado \n Iniciando trajeto')
            flagQRCode = False

            if (dm.timeMin < time.time() - dm.tSendMsgCAN):      #intervalo de tempo para realizar o processamento
                #print('inicio de do loop')
                dm.tSendMsgCAN = time.time()               #atualiza o tempo para o proximo intervalo
                angDir = nodeDecisionMaker.msgSteering
                
                if(angDir < dm.angMin):
                    angDir = dm.angMin
                    
                elif(angDir > dm.angMax):
                    angDir = dm.angMax

                #Ajustar Angulo Direcao
                msgCanId = 0x82
                param = [angDir]
                vc.sendMsg(s, msgCanId, param)

                msgPlatoon = vc.logCanPlatoon(s)

                breaking = dm.AEB(nodeDecisionMaker.msg_depth)

                if(not breaking):    
                    
                    if msgPlatoon[0] != "":

                        print(msgPlatoon[0])
                            
                        ang = msgPlatoon[1]
                        rpm = msgPlatoon[2]
                        #print("Angulo Direcao: {}; RPM: {}".format(ang, rpm))

                        #Ajustar RPM do Platoon
                        msgCanId = 0x56
                        #[Direcao Esq, RPM Esq, #Direcao Dir, RPM Dir]
                        param = [1, rpm, 1, rpm]
                        vc.sendMsg(s, msgCanId, param)
            
                else:
                    pass
                    # cv2.imshow("Depth", depth.get_data())
                    # cv2.imshow('Resultado', img) 
                msgCanId = 0x94
                param = [2, 0, 0, 0, angDir, msgPlatoon[2], msgPlatoon[2]]
                vc.sendMsg(s, msgCanId, param)
                #param = [1, 1, 5, 3, angDir, dm.rpm_can, dm.rpm_can]
                #msgCanId = 0x91
                #vc.sendMsg(msgCanId, param)
            # cv2.waitKey(1)
            gc.collect()

            if(flagObjectYOLOReceived and (not flagQRCode)):
                try: 
                    jsonObjectYOLO = ast.literal_eval(nodeDecisionMaker.msgObjectYOLO.data)

                    for object in jsonObjectYOLO:
                        if(jsonObjectYOLO[object]['classId'] == "stop sign"):
                            print('Placa de PARE identificada \n Parando...')
                            msgCanId = 0x56
                            #[Direcao Esq, RPM Esq, #Direcao Dir, RPM Dir]
                            param = [1, 0, 1, 0]
                            vc.sendMsg(msgCanId, param)
                except Exception as ex:
                    print("Exception: {}".format(ex))
                    pass

        except Exception as ex:
                print("Exception: {}".format(ex))   
                pass
        
        except KeyboardInterrupt:
            print("Exception: KeyboardInterrupt") 
        
            #FREIA PID
            msgCanId = 0x56
            #[Direcao Esq, RPM Esq, #Direcao Dir, RPM Dir]
            param = [1, 0, 1, 0]
            vc.sendMsg(s, msgCanId, param)

            #Ajustar Angulo Direcao
            msgCanId = 0x82
            param = [0x19]
            vc.sendMsg(s, msgCanId, param)

            s.close()

            break

        except:
            print("F")
'''
def main2():
    global flagObjectYOLOReceived 
    print("O node Decision maker foi iniciado")
    rospy.init_node("Decision_maker")       #inicia o no ROS
    nodeDecisionMaker = NodeDecisionMaker()  

    while(not rospy.is_shutdown()):
        if(flagObjectYOLOReceived):
            try:    
                jsonObjectYOLO = ast.literal_eval(nodeDecisionMaker.msgObjectYOLO.data)
                for object in jsonObjectYOLO:
                    print("Objeto Yolo ({}):\n-classId: {}\n-coords: {}\n-conf: {}\n-distance: {}\n\n".format(object, jsonObjectYOLO[object]['classId'], jsonObjectYOLO[object]['coords'], jsonObjectYOLO[object]['conf'], jsonObjectYOLO[object]['distance']))
            except Exception as ex:
                print("Exception: {}".format(ex))
                pass

def main3():
    global flagObjectYOLOReceived 
    global flagQRCode 

    print("O node Decision maker foi iniciado")
    rospy.init_node("Decision_maker")       #inicia o no ROS
    nodeDecisionMaker = NodeDecisionMaker()

    while(not flagQRCode):      #aguarda o inicio
        print("Esperando flag que autoriza o inicio")
    
    print("QR Code detectado pode iniciar: ", nodeDecisionMaker.msgQRCode.data)
    print('qr code iderntificado \n Iniciando trajeto')
    msgCanId = 0x56
    #[Direcao Esq, RPM Esq, #Direcao Dir, RPM Dir]
    param = [1, 30, 1, 30]
    vc.sendMsg(msgCanId, param)
    flagQRCode = False

    while(not rospy.is_shutdown()):

        if(flagQRCode):
            print("QR Code detectado pode iniciar: ", nodeDecisionMaker.msgQRCode.data)
            print('qr code iderntificado \n Iniciando trajeto')
            msgCanId = 0x56
            #[Direcao Esq, RPM Esq, #Direcao Dir, RPM Dir]
            param = [1, 20, 1, 20]
            vc.sendMsg(msgCanId, param)
            flagQRCode = False

        if(flagObjectYOLOReceived and (not flagQRCode)):
            try: 
                jsonObjectYOLO = ast.literal_eval(nodeDecisionMaker.msgObjectYOLO.data)

                for object in jsonObjectYOLO:
                    if(jsonObjectYOLO[object]['classId'] == "stop sign"):
                        print('Placa de PARE identificada \n Parando...')
                        msgCanId = 0x56
                        #[Direcao Esq, RPM Esq, #Direcao Dir, RPM Dir]
                        param = [1, 0, 1, 0]
                        vc.sendMsg(msgCanId, param)

            except Exception as ex:
                print("Exception: {}".format(ex))
                pass
'''

if(__name__=='__main__'):
    main()
    #main2()
    #main3()
    

                