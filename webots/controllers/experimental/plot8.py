#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

def clean(a):
    a[a==-1] = np.nan

def plot(a, k):
    plt.subplot(8,1,k+1)
    plt.plot(a[:,k])
    plt.ylim((0, 2000))

gt = np.loadtxt('groundtruth.csv', delimiter=',')
#ss = np.loadtxt('simsens.csv')

clean(gt)

plot(gt, 0)
plot(gt, 1)
plot(gt, 2)
plot(gt, 3)
plot(gt, 4)
plot(gt, 5)
plot(gt, 6)
plot(gt, 7)

plt.xlabel('timestep')
#plt.ylabel('distance (mm)')
plt.show()

#plt.plot(ss)
#plt.legend(('groundtruth', 'simsens'))

