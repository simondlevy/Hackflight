#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt


# https://stackoverflow.com/questions/12998430/how-to-remove-xticks-from-a-plot
def hide_xticks():
    plt.tick_params(
        axis='x',          # changes apply to the x-axis
        which='both',      # both major and minor ticks are affected
        bottom=False,      # ticks along the bottom edge are off
        top=False,         # ticks along the top edge are off
        labelbottom=False) # labels along the bottom edge are off



def main():

    data = np.genfromtxt("altitude.csv", delimiter=",")

    time = data[:,0]

    targ = data[:,1]

    z = data[:,2]

    dz = data[:,3]

    thrust = data[:,4]

    plt.subplot(3, 1, 1)
    plt.plot(time, z)
    hide_xticks()
    plt.ylabel('z (m)')

    plt.subplot(3, 1, 2)
    plt.plot(time, dz, 'r')
    hide_xticks()
    plt.ylabel('dz/dt (m/s)')

    plt.subplot(3, 1, 3)
    plt.plot(time, thrust, 'k')
    plt.xlabel('time (sec)')
    plt.ylabel('thrust')

    plt.show()

main()
