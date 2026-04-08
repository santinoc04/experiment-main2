import Jetson.GPIO as GPIO
import rover_navigation.SonnyMath


class MyMotor:
    _encoder = None
    _encoderpin = 0
   
    _in1Pin = 0
    _in2Pin = 0
    _maxspeed = 0

    def __init__(self,EncoderPin,IN1Pin,IN2Pin,MAXSPEED):
        self._encoderpin = EncoderPin
        self._in1Pin = IN1Pin
        self._in2Pin = IN2Pin
        self._maxspeed = MAXSPEED
        GPIO.setup(IN1Pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(IN2Pin, GPIO.OUT, initial=GPIO.LOW)
       # GPIO.setup(EncoderPin, GPIO.OUT)
       # self._encoder = GPIO.PWM(EncoderPin,MAXSPEED)
       # self._encoder.start(100)
        print(f'Motor Created at pin {EncoderPin}')
            

    def MoveMotor(self, speed, forward):
        speed = speed*100
        if(speed < 50):
            dir1 = GPIO.LOW
            dir2 = GPIO.LOW
        else:

            if forward == True:
                dir1 = GPIO.HIGH
                dir2 = GPIO.LOW
            else:
                dir1 = GPIO.LOW
                dir2 = GPIO.HIGH
        
        GPIO.output(self._in1Pin, dir1)
        GPIO.output(self._in2Pin, dir2)
        #self._encoder.ChangeDutyCycle(speed)
        #print('Motor Moving')
    def StopMotor(self):
        #self._encoder.stop()
        dir1 = GPIO.LOW
        dir2 = GPIO.LOW
        GPIO.output(self._in1Pin, dir1)
        GPIO.output(self._in2Pin, dir2)
