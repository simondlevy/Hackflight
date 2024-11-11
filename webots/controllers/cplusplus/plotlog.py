#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

plt.subplot(2, 1, 1)
plt.plot(data[:,0])
#plt.ylim((-75,+75))
plt.ylabel('raw flow')

plt.subplot(2, 1, 2)
plt.plot(data[:,1], '-x')
plt.plot(data[:,2])
plt.ylabel('dy/dt')
plt.legend(('true', 'est'))

plt.show()

