#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

plt.plot(data[:,0] / 12.5 - 1.99)
plt.plot(data[:,1])
plt.legend(('count', 'action'))
plt.ylim((-1.2, +1.2))
plt.show()

