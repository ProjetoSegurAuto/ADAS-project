import socket
import time
import vector as vc

#HOST = "192.168.1.101"  # Standard loopback interface address (localhost)
#PORT = 2323  # Port to listen on (non-privileged ports are > 1023)
  
tSendMsgCAN = time.time()
angle_can = 15    
while True:
        try: 
                if 0.05 < time.time() - tSendMsgCAN:
                        #s.connect((HOST, PORT))
                        #msgCanId = 0x91
                        #param = [1, 1, 3, 1, 5, 1, 50, 50]

                        #param = [1, 2, 3, 4, 5, 6, 7]
                        msgCanId = 0x91
                        #msgCanId = 0x56
                        param = [1, 40, 1, 40]
                        vc.sendMsg(msgCanId, param)
                        vc.callLogCan()

        except KeyboardInterrupt:
                msgCanId = 0x82
                param = [25]
                #param = [1,0,1,0]
                vc.sendMsg(msgCanId, param)
                break

        #mesg= [1,8,0,0x5C,0,0,0,0,0,0,0,1,80,1,40]
        #msg = bytearray(mesg)
        #s.sendall(msg)

        #s.close()

        #while True:
        #    data = s.recv(14)
        #    if data[2] == 0x56:
        #        arquivo = open("dadosVelocidade.txt", "r")
        #        conteudo = arquivo.readlines()
        #        d = "rpmDireita," + str(data[11])+",rpmEsquerda," + str(data[9]) + "\n"
        #        conteudo.append(d)
        #        arquivo.close()
        #        arquivo = open("dadosVelocidade.txt", "w")
        #        #arquivo.writelines(conteudo)
        #        arquivo.close()
        #    if data[2] == 0x86:
        #        print (data)