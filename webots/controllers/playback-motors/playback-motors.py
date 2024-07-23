from controller import Robot

import numpy as np

def makeMotor(robot, name, direction):

    motor = robot.getDevice(name)

    motor.setPosition(float('+inf'))
    motor.setVelocity(direction)

    return motor

def main():

    arr = np.genfromtxt("motors.csv", delimiter=",")

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
            vel1, vel2, vel3, vel4 = arr[i]
            i+= 1

        m1.setVelocity(vel1)
        m2.setVelocity(vel2)
        m3.setVelocity(vel3)
        m4.setVelocity(vel4)

main()
