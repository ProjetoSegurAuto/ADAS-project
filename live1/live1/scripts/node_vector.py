import rospy
from std_msgs.msg import String

class Vector():
    def __init__(self):
        self.can_msg = str()
        self.pubCAN = rospy.Publisher('TPC11Communication', String ,queue_size=1)
        
    def pubCANMsg(self, can_msg):
        self.can_msg = can_msg 
        self.pubCAN.publish(String(self.can_msg))

def main():
    #Setup ROS
    rospy.init_node('intra-vehicular')                #inicia o Node
    rospy.loginfo('intra-vehicular was started!')
    
    #setup

    while not rospy.is_shutdown():          #Enquanto o ros não for fechado
        try:
            #loop
            pass       
        except Exception as e: 
            print(e) 

if __name__ == "__main__":
    main()