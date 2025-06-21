import os
os.system("sudo pigpiod")
from gpiozero import DistanceSensor, AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
import pigpio
import math
import matplotlib.pyplot as plt
from matplotlib import colors

sensorTrig = 4
sensorEcho = 17
servoPin = 27
stepperDIR = 23
stepperSTEP = 24

radius = []
theta = []
phi = []

my_factory = PiGPIOFactory()

sensor = DistanceSensor(
    sensorEcho,
    sensorTrig,
    pin_factory=my_factory
)

servo = AngularServo(
    servoPin,
    initial_angle=90,
    min_angle=0,
    max_angle=180,
    max_pulse_width=(2.5 / 1000),
    min_pulse_width=(0.5 / 1000),
    pin_factory=my_factory
)

pi = pigpio.pi()

if not pi.connected:
    print("Pigpio not connected. Is the daemon running?")
    exit()

pi.set_mode(stepperDIR, pigpio.OUTPUT)
pi.set_mode(stepperSTEP, pigpio.OUTPUT)


def distance():
    sensorDistance = sensor.distance * 100
    #sleep(0.06)
    print(f"Distance is {sensorDistance:.2f} [cm] away.")
    return sensorDistance


def servoTilt(angle):  #moves servo up by 1 degree
    servo.angle = angle
    sleep(.5)


def stepper_once(direction):
    global pi
    pi.write(stepperDIR, direction)
    pi.write(stepperSTEP, 1)
    sleep(.001)
    pi.write(stepperSTEP, 0)
    sleep(.001)


def echoThread():  #gear ratio is 4.05:1 and there's 200 steps. 4.05 times 200 is what == 810?
    direction = 1
    for angle in range(90, 160, 2):
        servoTilt(angle)
        if direction == 1:
            for steps in range(810):
                phi.append(math.radians(angle-35))
                theta.append(math.radians(steps / 2.25))
                radius.append(distance())
                stepper_once(direction)
        else:
            for steps in range(810, -1, -1):
                phi.append(math.radians(angle-35))
                theta.append(math.radians(steps / 2.25))
                radius.append(distance())
                stepper_once(direction)
        direction ^= 1
        print("Rotation Complete!")
        print("ServoTilt Complete!")


def cartesian(radius, theta, phi):
    points = []
    for radius, theta, phi in zip(radius, theta, phi):
        x = radius * math.cos(theta) * math.cos(phi)
        y = radius * math.sin(theta) * math.cos(phi)
        z = radius * math.sin(phi)

        points.append((x, y, z))
    return points



def graph(points, radius):
    x, y, z = zip(*points)
    r = radius[::5]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    norm = colors.Normalize(vmin=min(r), vmax=max(r))
    ax.scatter(x[::5], y[::5], z[::5], c=r, cmap='turbo', norm=norm, s=2)

    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_zlabel('Z (cm)')
    ax.set_title('Echolocation Graph')

    plt.show()


if __name__ == "__main__":
    print("game launch")
    try:
        echoThread()
        points = cartesian(radius, theta, phi)
        print("graphing!")
        graph(points, radius)

    except KeyboardInterrupt:
        print("smell ya later")

    finally:
        print("Goodbye")
        servo.close()
        os.system("sudo killall pigpiod")
        pi.stop()
