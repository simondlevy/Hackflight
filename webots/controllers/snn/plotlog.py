#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

plt.plot(data[:,0])
plt.plot(data[:,1])
plt.plot(data[:,2])

plt.show()

