#!/usr/bin/env python3

##########
#Informações sobre esse Node:
#Nome: modeling
#Descrição: modela o platoon

import rospy
from std_msgs.msg import Int32
import vector as vc
import dsu

class NodeModeling():
    def __init__(self):
        self.msgLeader = int()
        self.pubLeader = rospy.Publisher('TPC9Leader', Int32 ,queue_size=1)
        
    def pubMsgLeader(self, msg_leader):
        self.msgLeader = msg_leader #pegar o pai na dsu
        self.pubLeader.publish(Int32(self.msgLeader))

def main():
    #Setup ROS
    rospy.init_node('Modeling')                #inicia o Node
    rospy.loginfo('O node modeling foi iniciado!')
    
    MY_ID = 0 #altera para cada Orin

    nodeModeling = NodeModeling()                 #instanciando o objeto do No ros
    dsu_ = dsu.DSU()
    dsu_.dsBuild()

    #Dados do logger
    last_node = -1
    last_pos = -1
    last_action = -1

    curr_node = -1
    curr_pos = -1
    curr_action = -1

    while not rospy.is_shutdown():          #Enquanto o ros não for fechado
        try:
            #RECEBE VIA RF->VECTOR OS COMANDOS PARA UNIÃO
            can_msg = vc.logCANModeling(vc.openSocket())

            if(can_msg[0] != "" and can_msg[1] != "" and can_msg[2] != ""):
                curr_node = int(can_msg[0])
                curr_pos = int(can_msg[1]) 
                curr_action = int(can_msg[2])

            if(last_node != curr_node and last_pos != curr_pos and last_action != curr_action):
                if curr_action == 1:
                    car_u = dsu.Car(MY_ID, 0, 0) #minha posição é o segundo argumento, precisa implementar
                    car_v = dsu.Car(curr_node, curr_pos, 0)
                    dsu_.dsUnion(car_u, car_v)
                else:
                    car_u = dsu.Car(curr_node, curr_pos, 0)
                    dsu_.dsLeave(car_u)
    
                last_node = curr_node
                last_pos = curr_pos
                last_action = curr_action
                

            nodeModeling.pubMsgLeader(dsu_.dsFind(MY_ID))          
        except Exception as e: 
            print(e) 

if __name__ == "__main__":
    main()