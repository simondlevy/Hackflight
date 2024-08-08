#!/usr/bin/python3

import matplotlib.pyplot as plt


def main():

    time = []
    zsetpoint = []
    z = []
    motor_snn = []
    motor_target = []

    while True:

        words = []

        try:
            words = input().split()

        except:
            break

        if len(words) == 5:

            time.append(float(words[0]))
            zsetpoint.append(float(words[1]))
            z.append(float(words[2]))
            motor_snn.append(float(words[3]))
            motor_target.append(float(words[4]))

    plt.subplot(3, 1, 1)
    plt.plot(time, z)
    plt.plot(time, zsetpoint, 'r')
    plt.legend(['actual', 'setpoint'])
    plt.xticks([], [])
    plt.ylabel('z (m)')

    plt.subplot(3, 1, 3)
    plt.plot(time, motor_snn)
    plt.plot(time, motor_target)
    plt.legend(['SNN', 'target'])
    plt.xlabel('time (s)')
    plt.ylabel('motor (rad/s)')

    plt.show()


main()
