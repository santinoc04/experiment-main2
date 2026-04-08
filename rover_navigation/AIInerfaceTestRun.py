import AIInterface
import SonnyMath
import time

interface = AIInterface.AIInterface()
refreshtime = float(input("what is the re-evaluation time: "))
goalx = float(input("What is the x goal: "))
goaly = float(input("what is the y goal: "))
StartPosition = SonnyMath.Coordinates(0,0)
GoalPosition = SonnyMath.Coordinates(goalx,goaly)
print("\n-----------------------------------------------------------\n")
done = interface.IterateController(0,0,0,GoalPosition.X,GoalPosition.Y,False,True,True)
print("\n-----------------------------------------------------------\n")

while(not done):
    
    
   # interpVector
    done = interface.IterateController(0,0,0,GoalPosition.X,GoalPosition.Y,False,True,True)
    print("\n-----------------------------------------------------------\n")
    time.sleep(refreshtime)
        

