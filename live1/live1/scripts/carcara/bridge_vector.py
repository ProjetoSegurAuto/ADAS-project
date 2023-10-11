#!/usr/bin/env python3

##########
#Informações sobre esse Node:
#Nome: vector
#Descrição: Nó ROS que realiza a ponte entre a Orin e a Vector. Este nó surge da necessidade de outros nós, além do deciosion maker, ter acesso a comunicação intra-veicular. i) O dado enviados pela Orin é igual ao projeto original, entretanto deve ser alocado mais um espaço e o último elemento representa o ID da msg

import rospy
import vector as vc
from std_msgs.msg import Int64MultiArray, Float64MultiArray
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout
import sys


class SensorBox(QWidget):
    def __init__(self, title, *labels):
        super().__init__()

        self.sensor_layout = QVBoxLayout(self)

        self.title_label = QLabel(title)
        self.sensor_layout.addWidget(self.title_label)

        self.label_widgets = []
        for label_text in labels:
            label_widget = QLabel(label_text)
            self.sensor_layout.addWidget(label_widget)
            self.label_widgets.append(label_widget)

    def update_labels(self, labels):
        for i, label_text in enumerate(labels):
            if i < len(self.label_widgets):
                self.label_widgets[i].setText(label_text)

class MonitorWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("CAN Messages Monitoring")
        self.setGeometry(100, 100, 600, 400)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.layout = QVBoxLayout(self.central_widget)

        self.sensor_boxes = []
        self.create_sensor_box("Potenciômetro", "Setpoint", "Ângulo Atual", "Resistência Atual", "OK / ERROR", "Código de erro com descrição")
        self.create_sensor_box("Encoder Direito", "Setpoint", "Leitura atual", "OK / ERROR", "Código de erro com descrição")
        self.create_sensor_box("Encoder Esquerdo", "Setpoint", "Leitura atual", "OK / ERROR", "Código de erro com descrição")

        #example:
        #new_label = ["New Setpoint 1", "New Ângulo Atual 1", "New Resistência Atual 1", "New OK / ERROR 1", "New Código de erro 1"]
        #self.update_label(self.sensor_boxes[0], new_label)


    def create_sensor_box(self, title, *labels):
        sensor_box = SensorBox(title, *labels)
        self.sensor_boxes.append(sensor_box)
        self.layout.addWidget(sensor_box)

    def update_label(self, sensor_box, new_label):
            sensor_box.update_labels(new_label)


class Bridge():
    def __init__(self):
        self.__can_message = Float64MultiArray()
        self.__orin_message = Int64MultiArray()
        self.__flagReceive = False
        self.socket = vc.openSocket()  

        self.subDataFromOrin = rospy.Subscriber('TPC10Decision_Maker', Int64MultiArray, self.callBackDataFromOrin)
        self.pubCAN = rospy.Publisher('TPC9Bridge', Float64MultiArray , queue_size=1)
        self.pubCAN1 = rospy.Publisher('TPC10Bridge', Float64MultiArray , queue_size=1)
        
    def pubCANMessage(self, can_message):
        self.__can_message.data = can_message#can_message 
        print("mandei: ",end='')
        print(self.__can_message)
        self.pubCAN.publish(self.__can_message)
        self.pubCAN1.publish(self.__can_message)

    def callBackDataFromOrin(self, orin_message):
        print("callBackDataFromOrin")
        self.__flagReceive = True
        self.__orin_message = orin_message

    def getDataFromOrin(self) -> list:
        return list(self.__orin_message.data)
    
    def getFlagReceiveMessage(self) -> bool:
        return self.__flagReceive
    
    def setFlagReceiveMessage(self, value: bool):
        self.__flagReceive = value

def main():

    #Setup ROS
    rospy.init_node('bridge-communication')                #inicia o Node
    rospy.loginfo('the node bridge beetwen Orin and Vector Can-Bus was started!')
    
    #setup
    object_vector = Bridge()

    #Init GUI:
    app = QApplication(sys.argv)
    window = MonitorWindow()
    window.show()
    app.exec_()
    
    older_can_msg = list()

    # loop
    while not rospy.is_shutdown():  # Enquanto o ros não for fechado
        try:
            # recebimento [CAN -> ORIN]
            data_logger = vc.logCAN(object_vector.socket)
            if (data_logger != None and data_logger != ""):
                if data_logger[0] == 0x80:
                    byte_error = data_logger[5]
                    if (byte_error == 225):
                        has_error = "ERROR"
                        str_error = str(byte_error) + " - " + 'Fim de curso ESQUERDO nao detectado'
                    elif (byte_error == 226):
                        has_error = "ERROR"
                        str_error = str(byte_error) + " - " + 'Fim de curso ESQUERDO nao detectado '
                    elif (byte_error == 227):
                        has_error = "ERROR"
                        str_error = str(byte_error) + " - " + 'Sensor com valor minimo maior que o maximo'
                    elif (byte_error == 229):
                        has_error = "ERROR"
                        str_error = str(byte_error) + " - " + 'Sensor com valor minimo e maximo iguais'
                    elif (byte_error == 230):
                        has_error = "ERROR"
                        str_error = str(byte_error) + " - " + 'Nao ha diferenca entre o valor da posicao atual e maximo'
                    elif (byte_error == 231):
                        has_error = "ERROR"
                        str_error = str(byte_error) + " - " + 'Nao ha diferenca entre o valor da posicao atual e minimo'
                    elif (byte_error == 232):
                        has_error = "ERROR"
                        str_error = str(byte_error) + " - " + 'Rodas nao centralizadas durante a calibracao'
                    else:
                        has_error = "OK"
                        str_error = 'Sem erro'
                    new_label = [f"Setpoint: {data_logger[2]}", f"Ângulo Atual: {data_logger[1]}",
                                 f"Resistência Atual :{str(data_logger[3]) + str(data_logger[4])}", has_error,
                                 str_error]
                    window.update_label(window.sensor_boxes[0], new_label)
                if data_logger[0] == 0x50:
                    byte_error = data_logger[7]
                    if (byte_error == 17):
                        has_error = "ERROR"
                        str_error = str(byte_error) + " - " + 'Roda girando para frente'
                    elif (byte_error == 18):
                        has_error = "ERROR"
                        str_error = str(byte_error) + " - " + 'Roda ESQUERDA girando para frente e roda DIREITA girando para tras'
                    elif (byte_error == 33):
                        has_error = "ERROR"
                        str_error = str(byte_error) + " - " + 'Roda ESQUERDA girando para tras e roda DIREITA girando para frente'
                    elif (byte_error == 34):
                        has_error = "ERROR"
                        str_error = str(byte_error) + " - " + 'Rodas girando para tras'
                    elif (byte_error == 225):
                        has_error = "ERROR"
                        str_error = str(byte_error) + " - " + 'sensor ESQUERDO sem leitura'
                    elif (byte_error == 226):
                        has_error = "ERROR"
                        str_error = str(byte_error) + " - " + 'sensor DIREITO sem leitura'
                    elif (byte_error == 227):
                        has_error = "ERROR"
                        str_error = str(byte_error) + " - " + 'ambos sensores sem leitura'
                    else:
                        has_error = "OK"
                        str_error = 'Sem erro'
                    new_label_dir = [f"Setpoint: {data_logger[6]}", f"Leitura Atual: {data_logger[5]}",
                                     has_error, str_error]
                    new_label_esq = [f"Setpoint: {data_logger[4]}", f"Leitura Atual: {data_logger[3]}",
                                     has_error, str_error]
                    window.update_label(window.sensor_boxes[1], new_label_dir)
                    window.update_label(window.sensor_boxes[2], new_label_esq)

                object_vector.pubCANMessage(data_logger)

            # envio [ORIN -> CAN]
            if (object_vector.getFlagReceiveMessage()):
                curr_data = object_vector.getDataFromOrin()
                print(curr_data)
                vc.sendMsg(object_vector.socket, curr_data[len(curr_data) - 1], curr_data[:len(curr_data) - 1])
                object_vector.setFlagReceiveMessage(False)

        except Exception as e:
            print(e) 

if __name__ == "__main__":
    main()
