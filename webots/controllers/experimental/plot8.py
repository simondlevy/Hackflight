#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

def plot(gt, ss, k):
    plt.subplot(8,1,k+1)
    plt.plot(gt[:,k])
    plt.plot(ss[:,k])
    plt.ylim((0, 2000))
    if k == 0:
        plt.legend(('groundtruth', 'simsens'))

def loadcsv(filename):
    a = np.loadtxt(filename, delimiter=',')
    a[a==-1] = np.nan
    return a

gt = loadcsv('groundtruth.csv')
ss = loadcsv('simsens.csv')

plot(gt, ss, 0)
plot(gt, ss, 1)
plot(gt, ss, 2)
plot(gt, ss, 3)
plot(gt, ss, 4)
plot(gt, ss, 5)
plot(gt, ss, 6)
plot(gt, ss, 7)

plt.xlabel('timestep')
#plt.ylabel('distance (mm)')
plt.show()

#plt.plot(ss)
#plt.legend(('groundtruth', 'simsens'))

