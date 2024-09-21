#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',', skiprows=1)

time = data[:,0]

stick = data[:,1]

dydt = data[:,2]

output = data[:,3]

plt.plot(time, stick)
plt.plot(time, output)
plt.legend(['stick', 'output'])
#plt.plot(time, demand2)
#plt.plot(time, demand3)
#plt.legend(['demand1', 'demand2', 'demand3'])
plt.xlabel('time (s)')

plt.show()

