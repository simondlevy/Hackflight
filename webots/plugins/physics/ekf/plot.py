#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

plt.subplot(3, 1, 1)
plt.plot(data[:,0])
plt.ylabel('flow')

plt.subplot(3, 1, 2)
plt.plot(data[:,1])
plt.ylabel('est')

plt.subplot(3, 1, 3)
plt.plot(data[:,2])
plt.ylabel('true')

# plt.legend(('est', 'true'))

plt.show()

