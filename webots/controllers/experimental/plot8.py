#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

def clean(a):
    a[a==-1] = np.nan

def plot(a, k):
    plt.subplot(8,1,k+1)
    plt.plot(a[:,k])

gt = np.loadtxt('groundtruth.csv', delimiter=',')
#ss = np.loadtxt('simsens.csv')

clean(gt)

plot(gt, 0)

plt.ylim((0, 4000))
plt.xlabel('timestep')
plt.ylabel('distance (mm)')
plt.show()

#plt.plot(ss)
#plt.legend(('groundtruth', 'simsens'))

