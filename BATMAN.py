import os
os.system("sudo pigpiod")
from gpiozero import DistanceSensor, OutputDevice, AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep, time, strftime
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#------------------------------------ECHO------------------------------------

trigPin = 5
echoPin = 6

my_factory = PiGPIOFactory()
sensor = DistanceSensor(echo=echoPin, trigger=trigPin, max_distance=4, pin_factory=my_factory)  #4 meters

motorPins = (18, 23, 24, 25)  #should be on the top right of the breadboard
motors = list(map(lambda pin: OutputDevice(pin, pin_factory=my_factory), motorPins))
CCWStep = (0x01, 0x02, 0x04, 0x08)
CWStep = (0x08,0x04,0x02,0x01)


myGPIO = 17
servoDelay = 0.02
correction = 0.0

maxPW=(2.4+correction)/1000
minPW=(0.5-correction)/1000

servo = AngularServo(myGPIO, initial_angle=0, min_angle=0, max_angle=180, min_pulse_width=minPW, max_pulse_width=maxPW, pin_factory=my_factory)



def ultraSonic():
    last = -1
    dist = round(sensor.distance * 100, 4)
    if dist != last:
        print(f"[{strftime('%H:%M:%S')}] Distance: {dist} cm")

def onePeriod(direction, ms):
    for j in range(len(CCWStep)):  #four steps per cycle
        for i in range(len(motorPins)):  #four pins
            if direction == 1:
                motors[i].on() if (CCWStep[j] == 1 << i) else motors[i].off()
            else:
                motors[i].on() if (CWStep[j] == 1 << i) else motors[i].off()

        if ms < 3: ms = 3  #at any point if ms drops below three, ____ motor will rotate inaccurately
        sleep(ms * 0.001)
    ultraSonic()

def moveSteps(ms, steps):
    boolean = True
    polarCoords = []
    angle = 0
    direction = 0

    servo.angle = 0
    sleep(1.5)

    for deg in range(0, 181):
        servo.angle = deg
        for i in range(steps):
            onePeriod(direction, ms)
            angle += (360 / steps)
            dist = sensor.distance * 100
            polarCoords.append((dist, angle, deg))
            print(f"Step {i}: {dist:.4f} cm, Angle: {angle:.4f}")
        if boolean == True:
            direction = 1
            boolean = False

        elif boolean == False:
            direction = 0
            boolean = True

        print(f"Vertical Angle {deg} done.")
        sleep(1)
        
    return polarCoords


def motorStop():
    for i in range(0, 4):
        motors[i].off()


def stepper():
        polar = moveSteps(3, 512)
        sleep(0.5)
        return polar



if __name__ == '__main__':
    print("Game Start!")
    try:


        polar = stepper()

        cartesian = [(r * math.cos(math.radians(theta)) * math.cos(math.radians(phi)), r * math.sin(math.radians(theta)) * math.cos(math.radians(phi)), r * math.sin(math.radians(phi))) for r, theta, phi in polar]

        x = [x for x, y, z in cartesian]
        y = [y for x, y, z  in cartesian]
        z = [z for x, y, z in cartesian]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.scatter(x, y, z, c='b', marker='o')

        ax.set_title("3D Scan")
        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.set_zlabel('Z Axis')

        plt.show()


    except KeyboardInterrupt:
        print("Goodbye... ")
    finally:
        sensor.close()
        motorStop()
        os.system("sudo killall pigpiod")


