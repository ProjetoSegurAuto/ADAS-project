import time
import vector_teste as vc

#HOST = "192.168.1.101"  # Standard loopback interface address (localhost)
#PORT = 2323  # Port to listen on (non-privileged ports are > 1023)

s = vc.openSocket()
#print(s)
tSendMsgCAN = time.time()  
angDir = 1
rpmCan = 30

while True:
        try: 
                if 0.05 < time.time() - tSendMsgCAN:
                        # msgCanId = 0x56
                        # param = [1, rpmCan, 1, rpmCan]
                        #msgCanId = 0x82
                        #param = [angDir]
                        #msgCanId = 0x94
                        #param = [3, 0, 0, 0, angDir, rpmCan, rpmCan]
                        msgCanId = 0x97
                        param = [3, 1, 1, 1, 1, 1, 1, 1]
                        print(param)
                        vc.sendMsg(s, msgCanId, param)
                        time.sleep(1)
                        #msgPlatoon, retornoAng, retornoRPM = vc.logCanPlatoon(s)
                        #print(msgPlatoon)
                        #vc.logCAN(s)


        except KeyboardInterrupt:
                #msgCanId = 0x82
                #param = [25]

                msgCanId = 0x56
                param = [1,0,1,0]

                vc.sendMsg(s, msgCanId, param)
                break
