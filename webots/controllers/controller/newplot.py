#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

standard = np.loadtxt('standard.csv', skiprows=6 )
snn = np.loadtxt('snn.csv', skiprows=6)
plt.plot(standard)
plt.plot(snn)
plt.legend(('standard', 'snn'))
plt.show()


