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

plt.subplot(2,1,1)
plt.plot(data[:,0])
plt.plot(data[:,1])
plt.plot(data[:,3])
plt.legend(('throttle', 'dz/dt', 'action'))

plt.subplot(2,1,2)
plt.plot(data[:,2])
plt.legend(('counts'))

plt.show()

