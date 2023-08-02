#!/usr/bin/env python3

##########
#Informações sobre esse Node:
#Nome: modeling
#Descrição: modela o platoon

import rospy
import dsu

class NodeModeling():
    def __init__(self):
        self.msgLeader = int()
        self.pubLeader = rospy.Publisher('TPC9Leader', int ,queue_size=1)
        
    def pubMsgLeader(self, msg_leader):
        self.msgLeader = msg_leader #pegar o pai na dsu
        self.pubLeader.publish(self.msgLeader)

def main():
    #Setup ROS
    rospy.init_node('Modeling')                #inicia o Node
    rospy.loginfo('O node modeling foi iniciado!')
    
    HOW_MANY_CARS = 2
    MY_ID = 0 #altera para cada Orin

    nodeModeling = NodeModeling()                 #instanciando o objeto do No ros
    dsu_ = dsu.DSU(HOW_MANY_CARS)

    while not rospy.is_shutdown():          #Enquanto o ros não for fechado
        try:
            #RECEBE VIA RF->VECTOR OS COMANDOS PARA UNIÃO

            nodeModeling.pubMsgLeader(dsu_.dsFind(MY_ID))          
        except Exception as e: 
            print(e) 

if __name__ == "__main__":
    main()