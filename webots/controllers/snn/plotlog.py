#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

plt.plot(data[:,0])
plt.plot(data[:,1])
plt.legend(['approx', 'actual'])
plt.ylim([-1,+1])

plt.show()
