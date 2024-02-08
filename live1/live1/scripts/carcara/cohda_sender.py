import socket
import struct
import time

class Send2Cohda():
    def __init__(self, HOST, PORT) -> None:
        self.SERVER_HOST = HOST
        self.SERVER_PORT = PORT

    def sendPacket(self, packet):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            print("Tentando conectar...")
            client_socket.connect((self.SERVER_HOST, self.SERVER_PORT))
            print("Conexão OK")

            # ID da mensagem
            priority = packet[0]

            # Adiciona a prioridade no início da mensagem
            packet_with_priority = packet + [priority]

            # Serializa os dados usando struct
            packed_data = struct.pack(f"{len(packet) + 1}i", *packet_with_priority)
            client_socket.sendall(packed_data)
            print(f"Enviado: {packet}")

            # Aguarda pelo ACK do servidor
            ack = client_socket.recv(1024).decode('utf-8')
            if ack == "ACK":
                print("Recebido ACK do servidor")
            else:
                print("Erro: ACK não recebido")

        except Exception as e:
            print(f"Erro ao enviar: {str(e)}")
        finally:
            client_socket.close()
