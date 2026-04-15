import rover_navigation.PIDController as PIDController
import rover_navigation.SonnyMath as SonnyMath
import math
import rover_navigation.MovementSystem as MovementSystem
import time

class AIInterface:
    #CONSTANTS
    ang_kp = .8
    ang_ki = 0.1
    ang_kd = 0.1
    dist_kp = 1
    dist_ki = 0.1
    dist_kd = 0.1

    errang_kp = 0.1
    errang_ki = 0.1
    errang_kd = 0.1
    errdist_kp = 0.1
    errdist_ki = 0.1
    errdist_kd = 0.1
    pi = 3.1415

    #motor turn speed is 1 rotation every 1.81 seconds radius is 6 cm = 6/100m
    TurnFreq = 1/9.3
    WhelR = 6/100
    AxleLength = 37.5/100

    #WhelAngVel = TurnFreq*2*pi
    WhelAngVel = 2*pi/1.81
    MaxSpeed = WhelAngVel*WhelR
    MaxTurnAngSpeed = 2*pi*TurnFreq



    normalizingDistance = 0.0254 # in meters. Anything greater than this distance will warrant the maximum speed for traversal
    normalizingAngle = pi/8 # in radians. Anything greater than this angle will warrant the maximum speed for angular change
    distTolerance = 0.0254 #distance at which it wont rotate or move forward anymore

 
 

    def __init__(self):
       #Distance, Angle, Quad = self._CalculateDeltas(self._goalCoords,self._currentCoords)
       self._dt = 1
       self._goalCoords: SonnyMath.Coordinates = SonnyMath.Coordinates(0,0)
       self._currentCoords: SonnyMath.Coordinates = SonnyMath.Coordinates(0,0)
       self._prevorientationAngle = 0
       self.firstRun = True
       self._prevTime = time.time()
       self.movementcontroller: MovementSystem.MovementController = MovementSystem.MovementController()
       self._prevMotorandAngOutput: SonnyMath.Coordinates =  SonnyMath.Coordinates(0,0)#Motor x. ang y
       self._angPID = PIDController.PIDController(0,0,self._dt,self.ang_kp,self.ang_ki,self.ang_kd)
       self._distPID = PIDController.PIDController(0,0,self._dt,self.dist_kp,self.dist_ki,self.dist_kd)
       self._errangPID = PIDController.PIDController(0,0,self._dt,self.errang_kp,self.errang_ki,self.errang_kd)
       self._errdistPID = PIDController.PIDController(0,0,self._dt,self.errdist_kp,self.errdist_ki,self.errdist_kd)
       

    def _GuessPosition(self, RoverisOrigin: bool):
        angPerc = self._prevMotorandAngOutput.Y/(self.pi/2)
        angvel = angPerc*self.MaxTurnAngSpeed
       # if(self._prevMotorandAngOutput.Y > )
        vel = self._prevMotorandAngOutput.X*self.MaxSpeed
        print(f"Vel ----- {vel}")
        dx = 0
        dy = 0
        theta =0

        dx2 = 0
        dy2 = 0

        dtheta = angvel*self._dt
        if(RoverisOrigin):
            R, Angle, Vec = self._CalculateDeltas(self._goalCoords,self._currentCoords)
            nAngle = dtheta+Angle
            dy2 = R*math.cos(-nAngle)-R*math.cos(-Angle)
            dx2 = R*math.sin(-nAngle)-R*math.sin(-Angle)
        
        if(abs(self._prevMotorandAngOutput.Y) == self.pi/2):
            dx = 0
            dy = 0
            theta =0
            vel = 0
        elif(angvel == 0):
            dy = vel*self._dt
            theta = 0
        elif():
            
            r = vel/angvel
            theta = vel*self._dt/r
            dx = -vel*self._dt*math.sin(theta) +dx2
            dy = vel*math.cos(theta) + dy2
        
        if(not RoverisOrigin):
            gang = self._prevorientationAngle+dtheta
            x = dx*math.cos(-gang) + dy*math.sin(-gang) + self._currentCoords.X 
            y = -dx*math.sin(-gang) + dy*math.cos(-gang)+ self._currentCoords.Y
            gpos = SonnyMath.Coordinates(x,y)
        else:
            gang = self._prevorientationAngle-dtheta
            x = -(dx*math.cos(-gang) + dy*math.sin(-gang)) + self._goalCoords.X 
            y = -(-dx*math.sin(-gang) + dy*math.cos(-gang))+ self._goalCoords.Y
            gpos = SonnyMath.Coordinates(x,y)
            gang = 0
        
        #limit angle between -pi/2 and pi/2
      #  print(f"Guess function: pr mod gang: {gang} dtheta {dtheta}")
        intmod = math.floor(gang /(2*self.pi))
        gang = gang-intmod*2*self.pi

        gang = SonnyMath.SonnyMath.limitAngle(gang)     
    
      #  print(f"dt: {self._dt} dx {dx} dy {dy} x {x} y {y}")
        return gpos, gang
    def _CalculateDt(self):
        t = time.time()
        self._dt = t-self._prevTime
        self._prevTime = t
    def FollowPath(self, path,sleep):
        i = 1
        refreshError = False
        done = False
        print(f"{path}")
        while i < len(path):
            print(f"Next position x: {path[i][0]} y: {path[i][1]}")
            while done == False:
                done = self.IterateController(path[0][0],path[0][1],0,path[i][0], path[i][1],refreshError,False,True)
                if done:
                    refreshError = True
                    print(f"Position reached moving to next position")
                else:
                    refreshError = False
                time.sleep(sleep)
            done = False
            i = i+1
    def GetCurrentHeading(self) -> float:
    
        #Return controller-estimated rover heading in radians.
    
        return float(self._prevorientationAngle)
    def littlestupidfunction(path):
        return "work...please?"
    def IterateController(self,currentPositionX: float, currentPositionY: float, currentAng: float,goalPositionX: float,goalPositionY: float,RefreshError: bool, RoverisOrigin: bool, _guessispos):
        done = False
        self._CalculateDt()
        if(RefreshError):
            self._angPID.ReserError()
            self._distPID.ReserError()
            self._errangPID.ReserError()
            self._errdistPID.ReserError()
        givenCurrentPos = SonnyMath.Coordinates(currentPositionX,currentPositionY)
        givenGoalCoords = SonnyMath.Coordinates(goalPositionX,goalPositionY)
        if(self.firstRun):
            print(f"set current to given current position {givenCurrentPos.X} {givenCurrentPos.Y}")
            self._currentCoords = givenCurrentPos
            self._goalCoords = givenGoalCoords
            self.firstRun = False
            self._dt = 1
  
        Quad: SonnyMath.Coordinates = SonnyMath.Coordinates(1,1)

        gpos, gang = self._GuessPosition(RoverisOrigin)

        if(_guessispos):
            if(not RoverisOrigin):
                givenCurrentPos = gpos
            else:
                givenGoalCoords = gpos
            currentAng = gang
        self._currentCoords = givenCurrentPos
        self._prevorientationAngle = currentAng
        self._goalCoords = givenGoalCoords
        print(f"x:{self._currentCoords.X} y:{self._currentCoords.Y} ang: {currentAng}, Xg: {self._goalCoords.X}, Yg: {self._goalCoords.Y}")
# for error
        errDistance, errVecAngle, errQuad = self._CalculateDeltas(gpos,self._currentCoords)
        errAngle = errVecAngle-self._prevorientationAngle

        errsign = errQuad.X
        erraut = errsign*self._errangPID.IterPID(errAngle,0.0)
      #  print(f"Error Dist, Error Angle, Current angle {errDistance,errAngle,currentAng}")
        errdut = self._errdistPID.IterPID(0.0,errDistance)


#for goal position
        Distance, VectorAngle, Quad = self._CalculateDeltas(self._goalCoords,self._currentCoords)

       # Angle = VectorAngle - self._prevorientationAngle
       # sign = Quad.X

        aut = self._angPID.IterPID(currentAng,VectorAngle)+erraut
       # aut = VectorAngle - currentAng+erraut
        print(f" Dist,  Angle, Current angle {Distance,VectorAngle,currentAng}")
        dut = self._distPID.IterPID(0,Distance)+errdut
        if Distance < self.distTolerance:
            dut = 0
            aut = 0
            done = True

      #  print(f"erraut {erraut}, errdut {errdut}, aut {aut}, dut {dut}, distance {Distance}, Vector angle {VectorAngle}")

        if(dut > self.normalizingDistance):
            dut = self.normalizingDistance
        if(aut > self.normalizingAngle):
            aut = self.normalizingAngle
        if(aut < -self.normalizingAngle):
            aut= -self.normalizingAngle
        
        percaut = aut/self.normalizingAngle #these are percentages of maximum angle and distance
        percdut = dut/self.normalizingDistance

           ####################ROUND FOR NON PWM
        xvarperc = round(percdut)
        #yvarprc = round(percaut*2)/2# half turn
        yvarprc = round(percaut)# no half turn
      #  print(f"Yvarperc {yvarprc}, percaut {percaut}")#autosave test
    ###################
        mag = xvarperc
        ang = self.pi/2*yvarprc

        xvar = mag*math.sin(-ang)#*Quad.X
        yvar = mag*math.cos(ang)#*Quad.Y

    #    print(f"Line 160: mag {mag}, ang {ang}, sinang {math.sin(ang)}, cosang {math.cos(ang)}, qUAD: X: {Quad.X} Y: {Quad.Y}")

        
        #turn into unit circle coordinate
        outputCoords: SonnyMath.Coordinates = SonnyMath.Coordinates(xvar,yvar)
      #  print(f"output coords x {outputCoords.X}, Y {outputCoords.Y}")
        
        
        self._prevMotorandAngOutput = self.movementcontroller.ParseInput(outputCoords) #where it goes to the movement system
        print(f"Motor Output {self._prevMotorandAngOutput.X}, ang output {self._prevMotorandAngOutput.Y}")
        print('-----------------------------------------')
        return done
        
    @staticmethod
    def _CalculateDeltas(goalPosition: SonnyMath.Coordinates,currentPosition:  SonnyMath.Coordinates):
        deltaVector:SonnyMath.Coordinates = SonnyMath.SonnyMath.SubtractCoords(goalPosition,currentPosition)
        distance = deltaVector.GetMag()
        VectorAngle = deltaVector.GetAngle() #TODO make this angle on range from 360
        Quad: SonnyMath.Coordinates = SonnyMath.Coordinates(1,1)
        if(deltaVector.X > 0):
            VectorAngle = -VectorAngle
        VectorAngle = SonnyMath.SonnyMath.limitAngle(VectorAngle)
        #     Quad.X = Quad.X*-1
        # if(deltaVector.Y < 0):
        #     Quad.Y = Quad.Y*-1
        print(f"dVector: X: {deltaVector.X} Y: {deltaVector.Y}")
        print(f"Quadx: {Quad.X} Quady: {Quad.Y}")
       # if((Quad.X == -1 and Quad.Y==1) or (Quad.X == 1 and Quad.Y==-1)):
        #VectorAngle = -Quad.X*VectorAngle
        
        return distance, VectorAngle, Quad
