#!/usr/bin/env python3

##########
#Informações sobre esse Node:
#Nome: vector
#Descrição: Nó ROS que realiza a ponte entre a Orin e a Vector. Este nó surge da necessidade de outros nós, além do deciosion maker, ter acesso a comunicação intra-veicular. i) O dado enviados pela Orin é igual ao projeto original, entretanto deve ser alocado mais um espaço e o último elemento representa o ID da msg

import rospy
import vector as vc
from std_msgs.msg import Int64MultiArray, Float64MultiArray

class Bridge():
    def __init__(self):
        self.__can_message = Float64MultiArray()
        self.__orin_message = Int64MultiArray()
        self.__flagReceive = False
        self.socket = vc.openSocket()  

        self.subDataFromOrin = rospy.Subscriber('TPC10Decision_Maker', Int64MultiArray, self.callBackDataFromOrin)
        self.pubCAN = rospy.Publisher('TPC9Bridge', Float64MultiArray , queue_size=1)
        self.pubCAN1 = rospy.Publisher('TPC10Bridge', Float64MultiArray , queue_size=1)
        
    def pubCANMessage(self, can_message):
        self.__can_message.data = can_message 
        self.pubCAN.publish(self.__can_message)

    def callBackDataFromOrin(self, orin_message):
        print("callBackDataFromOrin")
        self.__flagReceive = True
        self.__orin_message = orin_message

    def getDataFromOrin(self) -> list:
        return list(self.__orin_message.data)
    
    def getFlagReceiveMessage(self) -> bool:
        return self.__flagReceive
    
    def setFlagReceiveMessage(self, value: bool):
        self.__flagReceive = value

def main():
    #Setup ROS
    rospy.init_node('bridge-communication')                #inicia o Node
    rospy.loginfo('the node bridge beetwen Orin and Vector Can-Bus was started!')
    
    #setup
    object_vector = Bridge()
    
    #loop
    while not rospy.is_shutdown():          #Enquanto o ros não for fechado
        try:
            #recebimento [CAN -> ORIN]       
            data_logger = vc.logCAN(object_vector.socket)
            if(data_logger != None and len(data_logger)):
                object_vector.pubCANMessage(data_logger)

            #envio [ORIN -> CAN]
            if(object_vector.getFlagReceiveMessage()):
                curr_data = object_vector.getDataFromOrin()
                print(curr_data)
                vc.sendMsg(object_vector.socket, curr_data[len(curr_data)-1], curr_data[:len(curr_data)-1])
                object_vector.setFlagReceiveMessage(False)

        except Exception as e: 
            print(e) 

if __name__ == "__main__":
    main()
