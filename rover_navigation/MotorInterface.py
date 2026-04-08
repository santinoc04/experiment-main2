#Motor Wires Help
#White is Encoder B output -> Green for wiring
#Yellow is Encoder A Output ->Yellow 
#Blue Encoder VCC -> Blue
#Gray Encoder Ground -> Gray
#Black Motor power -> black
#Red Motor power -> Red


#https://github.com/NVIDIA/jetson-gpio jetson gpio

#when in1 is set to high and in 2 is set to low, motor spins forward
#in1 set to low and in2 set to high, motor spins backward
#Ena on the hbridge controls the % of maximum speed

import Jetson.GPIO as GPIO
import rover_navigation.MyMotor  
import time
import atexit
class MotorInterface:
    
    H1_ENA = 15 #FL
    H1_IN1 = 12
    H1_IN2 =13
   # H1_ENB = 21 #FR UPDATE ALL B HBRIDGE STUFF REMOVED
    #H1_IN3 =  23
    #H1_IN4 = 22

    H2_ENA = 33 #BR
    H2_IN1 = 24
    H2_IN2 = 26
    # H2_IN1 = 37
    # H2_IN2 = 38
   # H2_IN3 = 31 
   # H2_IN4 = 32
    #H2_ENB = 33 #BL

    #Motor 1 Wires (Front L)
    MFL_ENA = 11
    MFL_ENB = 16

    #Motor 2 Wires (Front R)
    MFR_ENA = 18
    MFR_ENB = 19

    #Motor 3 Wires (Back R)
    MBR_ENA = 37
    MBR_ENB = 38

    #Motor 4 Wires (Back L)
    MBL_ENA = 35
    MBL_ENB = 36

#BCM Pins

#     H1_ENA = 22 #FL
#     H1_IN1 = 18
#     H1_IN2 =27
#    # H1_ENB = 21 #FR UPDATE ALL B HBRIDGE STUFF REMOVED
#     #H1_IN3 =  23
#     #H1_IN4 = 22

#     H2_ENA = 13 #BR
#     H2_IN1 = 8
#     H2_IN2 = 7
   # H2_IN3 = 31 
   # H2_IN4 = 32
    #H2_ENB = 33 #BL

    # #Motor 1 Wires (Front L)
    # MFL_ENA = 11
    # MFL_ENB = 16

    # #Motor 2 Wires (Front R)
    # MFR_ENA = 18
    # MFR_ENB = 19

    # #Motor 3 Wires (Back R)
    # MBR_ENA = 37
    # MBR_ENB = 38

    # #Motor 4 Wires (Back L)
    # MBL_ENA = 35
    # MBL_ENB = 36

    MAXSPEED = 50 #in Hz
    #H1motorinputs = [H1_ENA, H1_IN1, H1_IN2, H1_ENB,H1_IN3,H1_IN4]
    #H2motorinputs = [H2_ENA, H2_IN1, H2_IN2, H2_ENB,H2_IN3,H2_IN4]
    H1motorinputs = [H1_ENA, H1_IN1, H1_IN2]
    H2motorinputs = [H2_ENA, H2_IN1]
    allMotorInputs =  H1motorinputs + H2motorinputs
    _Motors: list[rover_navigation.MyMotor.MyMotor]
   
    GPIO.setmode(GPIO.BOARD)


    _MotorFL = rover_navigation.MyMotor.MyMotor(H1_ENA,H1_IN1,H1_IN2,MAXSPEED)
   # _MotorFR = MyMotor.MyMotor(H1_ENB,H1_IN3,H1_IN4,MAXSPEED)
    _MotorBR = rover_navigation.MyMotor.MyMotor(H2_ENA,H2_IN1,H2_IN2,MAXSPEED)
  #  _MotorBL = MyMotor.MyMotor(H2_ENB,H2_IN3,H1_IN4,MAXSPEED)

    #_Motors = [_MotorFL,_MotorFR,_MotorBR,_MotorBL]
    #_Motors = [_MotorFL,_MotorBR]
    _Motors = [_MotorFL,_MotorBR]


    
    def MoveAll(self,speed,forward):
        for motor in self._Motors:
            motor.MoveMotor(speed,forward)


    def MoveLefts(self,speed,forward):
        self._MotorFL.MoveMotor(speed,forward)
        #self._MotorBL.MoveMotor(speed,forward)

    def MoveRights(self,speed,forward):
       # self._MotorFR.MoveMotor(speed,forward)
        #print("Move Rights Called")
        self._MotorBR.MoveMotor(speed,forward)

    def Rotate(self, speed, right: bool):
        oppospeed = False
        if right:
            oppospeed = True
        self.MoveRights(speed,oppospeed)
        self.MoveLefts(speed,right)

    def StopAll(self):
        for motor in self._Motors:
            motor.StopMotor()
        GPIO.cleanup()
# val = 30
# mi = MotorInterface()
# try:
#     while val <100:
#         time.sleep(1)
#         forw = True
#         if(val>75):
#             forw = False
#         mi.MoveAll(val,forw)
#         val += 5
# finally:
#     mi.StopAll()


