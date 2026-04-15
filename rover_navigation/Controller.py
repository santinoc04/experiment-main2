import pygame
import rover_navigation.SonnyMath as SonnyMath
from rover_navigation.SonnyMath import Coordinates
import rover_navigation.MovementSystem as MovementSystem


# Initialize pygame and the joystick module
pygame.init()
pygame.joystick.init()

# Check for connected controllers
if pygame.joystick.get_count() == 0:
    print("No controllers connected!")
else:
    joystick = pygame.joystick.Joystick(0) # Use the first connected controller
    joystick.init()
    print(f"Controller connected: {joystick.get_name()}")

# Main loop to capture inputs
running = True
inputcoordinate: Coordinates = Coordinates(0,0)
controller = MovementSystem.MovementController()
#clock = pygame.time.Clock()
while running:
    

    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:
            if event.axis == 0:
                #print(f"Axis {event.axis} moved to {event.value}")     
                inputcoordinate.X = event.value       
            if  event.axis == 1:
                #print(f"Axis {event.axis} moved to {event.value}")      
                inputcoordinate.Y = -1.0*event.value     
        

        elif event.type == pygame.QUIT:
            running = False
        #print(f"Coords are {inputcoordinate.X} and {inputcoordinate.Y}")
        controller.ParseInput(inputcoordinate)
    #clock.tick(0.5)
    
