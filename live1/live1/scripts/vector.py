import socket
import time

def logCAN(s):
    msgECU = s.recv(14)
    log = {}
    
    if msgECU[2] == 0x80:
        log['ECU'] = 'Direcao'
        log['Angulo'] = msgECU[9]
        log['AnguloSp'] = msgECU[10]
        log['ResPot'] = str(msgECU[11])+str(msgECU[12])
        sinal = msgECU[13]
        if(sinal==225):
            log['Sinal'] = 'ERRO - Fim de curso ESQUERDO nao detectado'
        elif(sinal==226):
            log['Sinal'] = 'ERRO - Fim de curso ESQUERDO nao detectado '
        elif(sinal==227):
            log['Sinal'] = 'ERRO - Sensor com valor minimo maior que o maximo'
        elif(sinal==229):
            log['Sinal'] = 'ERRO - Sensor com valor minimo e maximo iguais'
        elif(sinal==230):
            log['Sinal'] = 'ERRO - Nao ha diferenca entre o valor da posicao atual e maximo'
        elif(sinal==231):
            log['Sinal'] = 'ERRO - Nao ha diferenca entre o valor da posicao atual e minimo'
        elif(sinal==232):
            log['Sinal'] = 'ERRO - Rodas nao centralizadas durante a calibracao'
        else:
            log['Sinal'] = 'Sem erro'
        log['Sinal'] = str(sinal)+" - "+log['Sinal']

    elif msgECU[2] == 0x50:
        log['ECU'] = 'Powertrain'
        log['Reservado'] = msgECU[6]
        log['PID'] = msgECU[8]
        log['rpmEsq'] = msgECU[9]
        log['rpmEsqSp'] = msgECU[10]
        log['rpmDir'] = msgECU[11]
        log['rpmDirSp'] = msgECU[12]
        sinal = msgECU[13]
        if(sinal==17):
            log['Sinal'] = 'Roda girando para frente'
        elif(sinal==18):
            log['Sinal'] = 'Roda ESQUERDA girando para frente e roda DIREITA girando para tras'
        elif(sinal==33):
            log['Sinal'] = 'Roda ESQUERDA girando para tras e roda DIREITA girando para frente'
        elif(sinal==34):
            log['Sinal'] = 'Rodas girando para tras'
        elif(sinal==225):
            log['Sinal'] = 'ERRO - sensor ESQUERDO sem leitura'
        elif(sinal==226):
            log['Sinal'] = 'ERRO - sensor DIREITO sem leitura'
        elif(sinal==227):
            log['Sinal'] = 'ERRO - ambos sensores sem leitura'
        else:
            log['Sinal'] = 'Sem erro'
        log['Sinal'] = str(sinal)+" - "+log['Sinal']

    elif ((msgECU[3] << 8) + msgECU[2]) == 0x701:
        log['ECU'] = 'Radar'
        log['msgCanId'] = (msgECU[3] << 8) +  msgECU[2]
        log['radarClusterId'] = msgECU[6]
        log['radarDistLong'] = ((((msgECU[7] << 8) + msgECU[8]) >> 3)*0.2) - 500
        log['radarDistLat'] = ((((msgECU[8] & 0x03) << 8) + msgECU[9])*0.2) - 102.3

    else:
        log['ECU'] = 'ZERO'
        log['Angulo'] = msgECU[1]
        log['idCar'] = msgECU[2]
        log['TipoMsg'] = msgECU[3]
        log['Action'] = msgECU[4] 

    retorno = ""
    for l in log:
        retorno = retorno + "{}: {} | ".format(l, log[l])
    print (retorno)
    #return retorno
    
def sendMsg(s, msgCANId, value):
    try:    
        #mesg= [1, 8, 0, 14, 5, 0, 0, 7, 8, 9, 10, 11, 12, 13, 14]

        mesg= [0, 8, 0, msgCANId, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        #----------------------ECU DIRECAO-------------------------------------
        if msgCANId == 0x82:     #Ajustar Angulo Direcao
            mesg[0] = 1          #CAN1
            mesg[14] = value[0]  #Set point 
            
        elif msgCANId == 0x84:   #Calibra a Direcao
            mesg[0] = 1          #CAN1

        elif msgCANId == 0x86:   #Le Angulo de Giro Atual
            mesg[0] = 1          #CAN1
            
        elif msgCANId == 0x88:   #Set os Ganhos do PID da direcao
            mesg[0]  = 1          #CAN1
            mesg[9]  = value[0]       #
            mesg[10] = value[1]      #Kp
            mesg[11] = value[2]      #
            mesg[12] = value[3]      #Ki
            mesg[13] = value[4]      #
            mesg[14] = value[5]      #Kd
            
        elif msgCANId == 0x90:   #Le os ganhos do PID - Atual
            mesg[0] = 1          #CAN1
        
        elif msgCANId == 0x92:   #Reset ECU Direcao
            mesg[0] = 1          #CAN1
            
        #---------------------ECU POWERTRAIN--------------------------------
        elif msgCANId == 0x52:       #Ajusta a Velocidade do motor esquerdo
            mesg[0] = 1              #CAN1
            mesg[13] = value[0]      #Direcao
            mesg[14] = value[1]      #RPM
            
        elif msgCANId == 0x54:       #Ajusta a Velocidade do motor Direito
            mesg[0] = 1              #CAN1
            mesg[13] = value[0]      #Direcao
            mesg[14] = value[1]      #RPM
            
        elif msgCANId == 0x56:       #Ajusta a Velocidade de Ambos os motores
            mesg[0] = 1              #CAN1
            mesg[11] = value[0]      #Direcao Esq
            mesg[12] = value[1]      #RPM Esq
            mesg[13] = value[2]      #Direcao Dir
            mesg[14] = value[3]      #RPM Dir
            
        elif msgCANId == 0x58:       #Ajusta o PWM do motor esquerdo
            mesg[0] = 1              #CAN1
            mesg[13] = value[0]      #Direcao
            mesg[14] = value[1]      #PWM
            
        elif msgCANId == 0x60:       #Ajusta o PWM do motor direito
            mesg[0] = 1              #CAN1
            mesg[13] = value[0]      #Direcao
            mesg[14] = value[1]      #PWM
            
        elif msgCANId == 0x5C:       #Ajusta o PWM de Ambos os motores
            mesg[0] = 1              #CAN1
            mesg[11] = value[0]      #Direcao Esq
            mesg[12] = value[1]      #PWM Esq
            mesg[13] = value[2]      #Direcao Dir
            mesg[14] = value[3]      #PWM Dir
        
        elif msgCANId == 0x64:   #Set os Ganhos do PID do Motor Esquerdo
            mesg[0] = 1              #CAN1
            mesg[9] =  value[0]      #
            mesg[10] = value[1]      #Kp
            mesg[11] = value[2]      #
            mesg[12] = value[3]      #Ki
            mesg[13] = value[4]      #
            mesg[14] = value[5]      #Kd
            
        elif msgCANId == 0x66:   #Set os Ganhos do PID do Motor Direito
            mesg[0] = 1              #CAN1

            mesg[9] =  value[0]      #
            mesg[9] =  value[0]      #
            mesg[10] = value[1]      #Kp
            mesg[11] = value[2]      #
            mesg[12] = value[3]      #Ki
            mesg[13] = value[4]      #
            mesg[14] = value[5]      #Kd

        #---------------------ECU COMUNICAÇÃO--------------------------------
        elif msgCANId == 0x91:       #Ajusta a Velocidade do motor esquerdo
            mesg[0]  = 1             #CAN1
            mesg[8]  = value[0]      #Direção
            mesg[9]  = value[1]      #RPM Dir
            mesg[10] = value[2]      #RPM Esq
            mesg[11] = value[3]      #
            mesg[12] = value[4]      #Ki
            mesg[13] = value[5]      #
            mesg[14] = value[6]      #Kd
                
        msg = bytearray(mesg)

        s.sendall(msg)

    except Exception as ex:
        
        if(ex.errno == 113):
            erro = "[Errno 113] No route to host"
        elif(ex.errno == 104):
            erro = "[Errno 104] Connection reset by peer"
        elif(ex.errno == 32):
            erro = "[Errno 32] Broken pipe"
        else:
            erro = "{}".format(ex)

        print("Exception: {}".format(erro))
        s.close()
        pass

def openSocket():
    HOST = "192.168.1.101"  # Standard loopback interface address (localhost)
    PORT = 2323  # Port to listen on (non-privileged ports are > 1023)
    s =  socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
    s.connect((HOST, PORT))
    return s
            
def logCanDir(msgECU):
    retornoDir = ""

    if msgECU[2] == 0x80:
        retornoDir = msgECU[9]

    return retornoDir