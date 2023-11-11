import serial
import time

class ArduinoServer:
    """Server class for communicating with arduino clients."""
    def __init__(self):
        self.arduino = serial.Serial(port="COM6", baudrate=9600, timeout=None)
    
    def send_data(self, target):
        print(bytes(f"{target}", "utf-8"))
        self.arduino.write(bytes(f"{target}", "utf-8"))
        print("wrote: ", target)
    

if __name__ == "__main__":
    server = ArduinoServer()
    time.sleep(3)
    
    server.send_data(200)
    while True:
        print(server.arduino.readline().decode().strip())
    