#goal is to smoothly navigate to goal location we're going to make a PID class so we can have a different PID
#controller for the rotation and the distance
import rover_navigation.SonnyMath as SonnyMath
class PIDController:
    
 

    def __init__(self,GoalPoint,StartingPoint,dt,kp,ki,kd):
        self._dt = dt
        self._kp = kp
        self._ki = ki
        self._kd  = kd
        self._goalPoint = GoalPoint
        self._currentPoint = StartingPoint
        self._errorData: list[float] = [0,0]
    
        self._goalPoint: float = SonnyMath.Coordinates(0,0)
        self._currentPoint: float = SonnyMath.Coordinates(0,0)
    

    def IterPID(self,CurrentPoint,goalPoint):
        self._currentPoint = CurrentPoint
        self._goalPoint = goalPoint
        self._CalcError()
        p = self._Pfunction(self._kp,self._errorData)
        i = self._Ifunction(self._ki,self._errorData,self._dt)
        d = self._Dfunction(self._kd,self._errorData,self._dt)
        ut = p+i+d
       # print(f"P {p}, I,{i}, D {d}")
        return ut
    def ReserError(self):
        self._errorData = [0,0]

    def _CalcError(self):
        er: float = self._goalPoint -self._currentPoint
        self._errorData.append(er)
       # print(f"Error: {er}")
       # print(f"Error Data {self._errorData}")
    @staticmethod
    def _Pfunction(kp:float,errordata: list[float]):
      
        
        Pe = kp*errordata[len(errordata)-1]

        return Pe
    @staticmethod
    def _Ifunction(ki:float,errordata: list[float],dt:float):
        Ie = ki*SonnyMath.SonnyMath.Integrate(errordata,dt)
        return Ie
    @staticmethod
    def _Dfunction(kd:float,errordata: list[float],dt:float):
        De = kd*SonnyMath.SonnyMath.Differentiate(errordata,dt)
        return De