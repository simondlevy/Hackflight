from controller import Supervisor

import numpy as np

LOGFILE = 'pose.csv'

def makeMotor(robot, name, direction):

    motor = robot.getDevice(name)

    motor.setPosition(float('+inf'))
    motor.setVelocity(direction)

    return motor


def angles_to_rotation(phi, theta, psi):

    angs = np.radians((phi, theta, psi))

    maxang = np.max(np.abs(angs))

    if maxang == 0:

        return [0, 0, 1, 0]

    signs = np.array(list(-1 if ang < 0 else +1 for ang in angs))

    fracs = np.sqrt(np.abs(angs) / maxang)

    rs = signs * fracs

    return [rs[0], rs[1], rs[2], maxang]


def main():

    arr = np.genfromtxt(LOGFILE, delimiter=",")

    robot = Supervisor()

    copter_node = robot.getFromDef('ROBOT')
    translation_field = copter_node.getField('translation')
    rotation_field = copter_node.getField('rotation')

    m1 = makeMotor(robot, 'motor1', +1)
    m2 = makeMotor(robot, 'motor2', -1)
    m3 = makeMotor(robot, 'motor3', +1)
    m4 = makeMotor(robot, 'motor4', -1)

    timestep = int(robot.getBasicTimeStep())

    for line in arr:

        x, y, z, phi, theta, psi, m1val, m2val, m3val, m4val = line

        robot.step(timestep)

        translation_field.setSFVec3f([x, y, z])

        rotation_field.setSFRotation(angles_to_rotation(phi, theta, psi))

        m1.setVelocity(+m1val)
        m2.setVelocity(-m2val)
        m3.setVelocity(+m3val)
        m4.setVelocity(-m4val)

main()
