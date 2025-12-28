#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

def loadcsv(filename):
    a = np.loadtxt(filename, delimiter=',')
    a[a==-1] = np.nan
    return a[0,:]

gt = loadcsv('groundtruth.csv')
ss = loadcsv('simsens.csv')

n = len(gt)

diff = ss - gt

x = np.linspace(-n, n, n)

plt.plot(x, diff)
plt.plot(x, (1.25*x)**2)
plt.show()

'''
plt.plot(gt[0,:], ss[0,:])
plt.xlim((0,4000))
plt.xlabel('groundtruth')
plt.ylabel('simsens')
plt.ylim((0,4000))
plt.show()
'''
