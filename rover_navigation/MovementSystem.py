import rover_navigation.MotorInterface as MotorInterface
#import numpy as np
import math
import atexit
import rover_navigation.SonnyMath as SonnyMath
from rover_navigation.SonnyMath import Coordinates

class MovementController:



    motorInterface = MotorInterface.MotorInterface()
    atexit.register(motorInterface.StopAll)
    _wheeldiameter = 0
    _roverwidth = 0
    _previousLocation: Coordinates = Coordinates(0,0)
    _goalLocation: Coordinates = Coordinates(0,0)
    pi = 3.1415

    # this is the thumbstick-like control system
    def ParseInput(self,speed_dir_coords: Coordinates): #The magnitude of this coordinate should be between 0 and 1 and centered around 0
        forward = True
        rightTurn = True
        forwardTurn = True
        reverse = False
        
        if(speed_dir_coords.Y<0):
            #forward = False
            #forwardTurn = False
            reverse = True
        

        totalspeed = speed_dir_coords.GetMag() 
        turnAngle = speed_dir_coords.GetAngle(not reverse)
       # print(f"Movement System: Coords {speed_dir_coords.X} {speed_dir_coords.Y}, calcualtd Angle {turnAngle}")
        
        outputTurnAndMove = SonnyMath.Coordinates(0,0)
        outputTurnAndMove.X = totalspeed
        outputTurnAndMove.Y = turnAngle
        
        if(speed_dir_coords.X>0):
            outputTurnAndMove.Y = -turnAngle
        
        if(totalspeed>1.0):
            #print("Error: Input speed and dir has magnitude greater than 1")
            totalspeed = 1
             
        if(abs(speed_dir_coords.X)>1 or abs(speed_dir_coords.Y)>1):
            #print("Error: X or Y value for input controller exceed the unit circle bounds")
            return
        if(speed_dir_coords.X<0):
            rightTurn = False
            turnAngle = abs(turnAngle)
        
        if(abs(turnAngle)>self.pi/4):
            forwardTurn = False # reverseTurn is putting the turning wheels in reverse for a faster turn
            turnAngle = self.pi/2-turnAngle
            #if(not forward):
             #   forwardTurn = True
        wheelTurnSpeed = totalspeed*math.cos(turnAngle*2)
        self.MoveMotorsFromInput(totalspeed,wheelTurnSpeed,forward,rightTurn,forwardTurn,reverse)
        return outputTurnAndMove
        


    def MoveMotorsFromInput(self,totalSpeed,wheelTurnSpeed,forward,rightTurn,forwardTurn,reverse):
        if(reverse):
            forward = not forward
            rightTurn = not rightTurn
            forwardTurn = not forwardTurn
        if(rightTurn):
            self.motorInterface.MoveLefts(totalSpeed,forward)
         #   print("Left - Forward:",forward,"speed",totalSpeed)
            self.motorInterface.MoveRights(wheelTurnSpeed,forwardTurn)
          #  print("Right - Forward:",forwardTurn,"speed",wheelTurnSpeed)
        else:
            self.motorInterface.MoveLefts(wheelTurnSpeed,forwardTurn)
           # print("Left - Forward:",forwardTurn,"speed",wheelTurnSpeed)
            self.motorInterface.MoveRights(totalSpeed,forward)
           # print("Right - Forward:",forward,"speed",totalSpeed)
    
    
        
       
        

    



    
