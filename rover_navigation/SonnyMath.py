import math
class Coordinates:
        X: float = 0
        Y:float = 0
        
        def __init__(self,x,y):
            self.X = x
            self.Y = y
        def GetMag(self):
            Magnitude = pow(pow(self.X,2)+pow(self.Y,2),0.5)
            return float(Magnitude)
        
        def GetAngle(self,forward: bool = True): #gets angle between  vector 0,1 since that is the forward vector
            refvector = Coordinates(0,1)
            sign = "+"
            if(forward == False):
                refvector = Coordinates(0,-1)
                sign = "-"
               
            theta = SonnyMath.GetAngle(self,refvector)
            #print(f'{theta*180/3.1415} {sign}')
            return float(theta)
        
class SonnyMath:
    def __init__(self):
         pass
    def SubtractCoords(A: Coordinates,B: Coordinates):
        subx = A.X - B.X
        suby = A.Y - B.Y
        return Coordinates(subx,suby)

    def dot(A: Coordinates, B: Coordinates):
            dot = A.X*B.X+A.Y*B.Y
            return float(dot)
    def GetAngle(A:Coordinates,B: Coordinates):#gets angle between two vectors.
        if(float(A.GetMag()) == float(0) or float(B.GetMag()) == float(0)):
            theta = 0
        else:
            costheta = SonnyMath.dot(A,B)/(A.GetMag()*B.GetMag())
            theta = math.acos(costheta)
        return float(theta)
    def Differentiate(data: list[float],dt):
         intdata = (data[len(data)-1] - data[len(data)-2])*dt
         return intdata
    def Integrate(data: list[float],dt):
         ddt = (data[len(data)-1] - data[len(data)-2])/dt
         return ddt
    
