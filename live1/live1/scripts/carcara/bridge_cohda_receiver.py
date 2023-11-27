#!/usr/bin/env python3

##########
#Informações sobre esse Node:
#Nome: bridge
#Descrição: Nó ROS de recebimento da vector

import rospy
import vector as vc
from std_msgs.msg import Int64MultiArray, Float64MultiArray

class BridgeReceiverByCohda():
    def __init__(self):
        self.__can_message = Float64MultiArray()
        self.socket = vc.openSocket()  

        self.pubCohda = rospy.Publisher('TPC90BridgeCohda', Float64MultiArray , queue_size=1)
        
    def pubCohdaMessage(self, cohda_message):
        self.pubCohda.publish(cohda_message)

def main():
    #recebimento [CAN -> ORIN] | OBS: REFAZER ESSA FUNÇÃO, POIS ESTÁ MUITO LENTO   
    #setup
    object_cohda = BridgeReceiverByCohda()
    
    while not rospy.is_shutdown(): 
        #Setup ROS
        rospy.init_node('bridge-communication: cohda2orin')                #inicia o Node
        rospy.loginfo('the node bridge beetwen Orin and any device was started!')  
        
        vector_data, priority = object_cohda.receivePacket()
        object_cohda.pubCohdaMessage(vector_data)


if __name__ == "__main__":
    main()