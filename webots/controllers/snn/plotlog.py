#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

'''
plt.plot(data[:,0] / 12.5 - 1.99)
plt.plot(data[:,1])
plt.legend(('count', 'action'))
plt.ylim((-1.2, +1.2))
'''

time = data[:,0]
setpoint = data[:,1]
dzdt = data[:,2]
trained_output = data[:,3]
trained_counts = data[:,4]
byhand_counts = data[:,5]

plt.subplot(2,1,1)
plt.plot(time, setpoint)
plt.plot(time, dzdt)
plt.plot(time, trained_output)
plt.legend(['throttle', 'dz/dt', 'action'])

plt.subplot(2,1,2)
plt.plot(time, trained_counts)
plt.plot(time, byhand_counts)
plt.legend(('trained', 'byhand'))
plt.xlabel('time (sec)')
plt.ylabel('counts')

plt.show()

