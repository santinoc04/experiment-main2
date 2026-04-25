import serial
import numpy as np
class ArduinoIMU:
    def __init__(self):
        self.COM = "COMX"
        self.baud = 9600

        pass
    def ListenForSerial(self):
        ser = serial.Serial(self.COM, self.baud)  # Replace COM3 with your port
        rows = []
        keepRead = True
        startRead = False
        while keepRead:
            line = ser.readline().decode().strip()
            if line == "--":

                startRead = True
            if startRead:

                if line == "---":
                    keepRead = False
                    
                    continue
                if line:
                    row = np.fromstring(line,sep=',')
                    rows.append(row)
        mat = np.vstack(rows)
        return mat
                   
        