#!/usr/bin/env python3

##########
#Informações sobre esse Node:
#Nome: bridge
#Descrição: Nó ROS de envio

import rospy
import vector as vc
from std_msgs.msg import Int64MultiArray, Float64MultiArray
from cohda_sender import Send2object_Cohda

class BridgeSender():
    def __init__(self):
        self.__orin_message = Int64MultiArray()
        self.__flagReceive = False
        self.socket = vc.openSocket()  

        self.subDataFromOrin = rospy.Subscriber('TPC10Decision_Maker', Int64MultiArray, self.callBackDataFromOrin)
        
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
    rospy.init_node('bridge-communication: orin2X')                #inicia o Node
    rospy.loginfo('the node bridge beetwen Orin and any device was started!')
    
    #setup
    object_vector = BridgeSender()
    object_cohda = Send2object_Cohda('192.168.1.10', 9000)

    #loop
    while not rospy.is_shutdown():          #Enquanto o ros não for fechado
        try:
            #envio [ORIN -> CAN] | [ORIN -> object_COHDA]
            if(object_vector.getFlagReceiveMessage()):
                curr_data = object_vector.getDataFromOrin()
                print(f"dado corrente: {curr_data}")
                if(curr_data[len(curr_data)-1] < 0x90):
                    vc.sendMsg(object_vector.socket, curr_data[len(curr_data)-1], curr_data[:len(curr_data)-1])
                else:
                    ID = curr_data.pop()
                    curr_data.insert(0, ID)
                    object_cohda.sendPacket(curr_data)
                object_vector.setFlagReceiveMessage(False)

        except Exception as e: 
            print(e) 

if __name__ == "__main__":
    main()
