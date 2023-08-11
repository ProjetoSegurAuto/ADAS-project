import rospy
import vector as vc
from std_msgs.msg import String, Float64MultiArray

class Vector():
    def __init__(self):
        self.can_msg = str()
        self.socket = vc.openSocket()  

        self.subImg = rospy.Subscriber('TPC13Decision_Maker', Float64MultiArray, self.callbackImagem)
        self.pubCAN = rospy.Publisher('TPC11Communication', String ,queue_size=1)
        
    def pubCANMsg(self, can_msg):
        self.can_msg = can_msg 
        self.pubCAN.publish(String(self.can_msg))

    def callBackData(self):
        pass

    def getData(self):
        pass

def main():
    #Setup ROS
    rospy.init_node('intra-vehicular')                #inicia o Node
    rospy.loginfo('intra-vehicular was started!')
    
    #setup
    last_data = list()
    curr_data = list()

    object_vector = Vector()
    
    #loop
    while not rospy.is_shutdown():          #Enquanto o ros não for fechado
        try:
            #recebimento         
            data_logger = vc.logCAN(object_vector.socket)
            if(data_logger != None or data_logger != ""):
                object_vector.pubCANMsg(data_logger)
            #envio
            curr_data = object_vector.getData()
            if(last_data != curr_data):
                vc.sendMsg(object_vector.socket, curr_data[:len(curr_data)-1], curr_data[len(curr_data)-1])
                last_data = curr_data       
        except Exception as e: 
            print(e) 

if __name__ == "__main__":
    main()