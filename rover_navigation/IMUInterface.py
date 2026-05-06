import rover_navigation.SonnyMath as SonnyMath
from rover_navigation.SonnyMath import Coordinates
import math
import numpy as np
import rover_navigation.ArduinoIMU as ArduinoIMU
class IMUInterface:
    def __init__(self):
        self.IMU: ArduinoIMU.ArduinoIMU = ArduinoIMU.ArduinoIMU()
    def GetPosVelYaw(self, prevpos: Coordinates ,prevvel: Coordinates):
        datamat = self.IMU.ListenForSerial()
        IMUaccelx = datamat[:,0].tolist()
        IMUaccely = datamat[:,1].tolist()
        IMUaccelz = datamat[:,2].tolist()
        Yaw = datamat[:,4].tolist()

        

        accelx = -IMUaccely
        accely = IMUaccelx

        dvx = np.array(SonnyMath.SonnyMath.Integrate(accelx))
        dvy = np.array(SonnyMath.SonnyMath.Integrate(accely))


        vx = prevvel.X + dvx
        vy = prevvel.Y + dvy

        dsxloc = np.array(SonnyMath.SonnyMath.Integrate(vx.tolist()))
        dsyloc = np.array(SonnyMath.SonnyMath.Integrate(vy.tolist()))

      
        
        x = dsxloc*np.cos(-Yaw) + dsyloc*np.sin(-Yaw) + prevpos.X 
        y = -dsxloc*math.sin(-Yaw) + dsyloc*math.cos(-Yaw)+ prevpos.Y
        pos = SonnyMath.Coordinates(x,y)
        vel = SonnyMath.Coordinates(vx,vy)
        Yaw = Yaw[-1]
        return pos, vel 