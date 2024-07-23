from controller import Robot
from mixers import runcf
import numpy as np

def makeMotor(robot, name, direction):

    motor = robot.getDevice(name)

    motor.setPosition(float('+inf'))
    motor.setVelocity(direction)

    return motor

def main():

    arr = np.genfromtxt("demands.csv", delimiter=",")

    # Read motors.csv into a numpy array

    robot = Robot()

    m1 = makeMotor(robot, 'motor1', +1)
    m2 = makeMotor(robot, 'motor2', -1)
    m3 = makeMotor(robot, 'motor3', +1)
    m4 = makeMotor(robot, 'motor4', -1)

    timestep = int(robot.getBasicTimeStep())

    i = 0

    while robot.step(timestep) != -1:

        if i < len(arr):
            demands = arr[i]
            i+= 1

        m1val, m2val, m3val, m4val = runcf(demands)

        m1.setVelocity(+m1val)
        m2.setVelocity(-m2val)
        m3.setVelocity(+m3val)
        m4.setVelocity(-m4val)

main()
