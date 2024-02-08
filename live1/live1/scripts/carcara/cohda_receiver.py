import socket
import struct

VECTOR_SIZE = 10  

class ReceiveByCohda():
    def __init__(self, HOST, PORT) -> None:
        self.SERVER_HOST = HOST
        self.SERVER_PORT = PORT

    def receivePacket(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.server_socket.bind((self.SERVER_HOST, self.SERVER_PORT))

        self.server_socket.listen(1)
        print("Waiting for any connection")

        self.client_socket, client_address = self.server_socket.accept()
        print("Connection established")

        data = self.client_socket.recv(1024)

        unpacked_data = struct.unpack(f'{VECTOR_SIZE}iI', data)

        cohda_data = unpacked_data[:VECTOR_SIZE]
        priority = unpacked_data[VECTOR_SIZE]

        print(f"Received vector data: {cohda_data}")
        print(f"Received priority: {priority}")

        self.client_socket.close()
        self.server_socket.close()

        return cohda_data, priority

# Example usage:
# HOST = '127.0.0.1' 
# PORT = 12345 

# receiver = ReceiveByCohda(HOST, PORT)
# cohda_data, priority = receiver.receivePacket()
