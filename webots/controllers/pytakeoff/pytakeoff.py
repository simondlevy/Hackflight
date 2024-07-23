from time import time
import numpy as np
import matplotlib.pyplot as plt

from controller import Supervisor, Robot, GPS

# Control constants
K1 = 56
K2 = 25
K3 = 2

ZTARGET = 0.2

DT = 0.01

PLOT_TIME_LIMIT = 7

def makeMotor(robot, name, direction):

    motor = robot.getDevice(name)

    motor.setPosition(float('+inf'))
    motor.setVelocity(100 * direction)

    return motor


def hide_xticks():
    '''
    https://stackoverflow.com/
      questions/12998430/how-to-remove-xticks-from-a-plot
    '''
    plt.tick_params(
        axis='x',
        which='both',
        bottom=False,
        top=False,
        labelbottom=False)


def plot(tic, z_history, dz_history, motor_history):

    plt.suptitle('K1=%3.1f   K2=%3.1f   K3=%3.1f   DT=%3.2f' %
                 (K1, K2, K3, DT))
    t = np.linspace(0, PLOT_TIME_LIMIT, tic)

    plt.subplot(3, 1, 1)
    plt.plot(t, z_history)
    plt.plot(t, ZTARGET * np.ones(tic), 'r')
    plt.ylabel('z (m)')
    plt.legend(['Actual', 'Target'])
    hide_xticks()

    plt.subplot(3, 1, 2)
    plt.plot(t, dz_history)
    plt.ylabel('dz/dt (m/sec)')
    hide_xticks()

    plt.subplot(3, 1, 3)
    plt.plot(t, motor_history)
    plt.xlabel('Time (sec)')
    plt.ylabel('Motor (rad/sec)')

    plt.show()


def main():

    robot = Robot()
    gps = GPS("gps")
    timestep = int(robot.getBasicTimeStep())
    gps.enable(timestep)

    m1 = makeMotor(robot, 'motor1', +1)
    m2 = makeMotor(robot, 'motor2', -1)
    m3 = makeMotor(robot, 'motor3', +1)
    m4 = makeMotor(robot, 'motor4', -1)

    tic = 0

    start_time = time()

    # Start with faked-up initial motor to animate motor spin-up
    motor = K2 + K3

    motor_history = []
    z_history = []
    dz_history = []
    plotted = False
    zprev = 0

    while robot.step(timestep) != -1:

        # Get current altitude
        z = gps.getValues()[2]

        # Do first-differencing to get climb rate from altitude and
        # delay
        dz = (z - zprev) / DT

        motor = K1 + K2 * (K3 * (ZTARGET - z) - dz)

        zprev = z

        if tic / timestep < PLOT_TIME_LIMIT:
            motor_history.append(motor)
            z_history.append(z)
            dz_history.append(dz)

        elif not plotted:

            plot(tic, z_history, dz_history, motor_history)
            plotted = True

        tic += 1

        # Spin the motors
        m1.setVelocity(+motor)
        m2.setVelocity(-motor)
        m3.setVelocity(+motor)
        m4.setVelocity(-motor)


main()
