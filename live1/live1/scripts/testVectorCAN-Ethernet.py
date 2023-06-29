import socket
import time
import vector as vc

#HOST = "192.168.1.101"  # Standard loopback interface address (localhost)
#PORT = 2323  # Port to listen on (non-privileged ports are > 1023)
  
tSendMsgCAN = time.time()  
angDir = 5
rpmCan = 50

while True:
        try: 
                if 0.005 < time.time() - tSendMsgCAN:
                        #msgCanId = 0x56
                        #param = [1, rpmCan, 1, rpmCan]
                        msgCanId = 0x82
                        param = [angDir]
                        vc.sendMsg(msgCanId, param)


        except KeyboardInterrupt:
                msgCanId = 0x82
                #msgCanId = 0x56
                param = [25]
                #param = [1,0,1,0]
                vc.sendMsg(msgCanId, param)
                break