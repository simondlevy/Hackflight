#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt



def main():

    data = np.loadtxt('tmp.csv')

    plt.plot((data[:-1] + data[1:]) / 2)

    plt.ylim((6,10))

    plt.show()


main()
