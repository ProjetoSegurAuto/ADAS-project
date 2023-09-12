#!/usr/bin/env python3

##########
#Informações sobre esse Node:
#Nome: modeling
#Descrição: modela o platoon

import dsu


def main():

    
    MY_ID = 0 #altera para cada Orin
              #instanciando o objeto do No ros
    dsu_ = dsu.DSU()
    dsu_.dsBuild()

    # # PARA TESTE !
    # car_u = dsu.Car(0, 10, 0) 
    # car_v = dsu.Car(1, 1, 0)
    # dsu_.dsUnion(car_u, car_v)
    # # TESTE !
    can_msg = list()

    #Dados do logger
    last_node = -1
    last_pos = -1
    last_action = -1

    curr_node = -1
    curr_pos = -1
    curr_action = -1

    while True:
        try:
            #RECEBE VIA RF->VECTOR OS COMANDOS PARA UNIÃO
            if(True):
                a = input()
                can_msg.append(a)
                a = input()
                can_msg.append(a)
                a = input()
                can_msg.append(a)
                
                #rever a separação da msg
                if len(can_msg) == 3:
                    curr_node = int(can_msg[0])
                    curr_pos = int(can_msg[1]) 
                    curr_action = int(can_msg[2])
                    print("ENTREI")
                    if(last_node != curr_node or last_pos != curr_pos or last_action != curr_action):
                        
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
                    
                can_msg.clear()
                print(f"my dad {dsu_.dsFind(MY_ID)}")
        
        except Exception as e: 
            print(e) 

if __name__ == "__main__":
    main()