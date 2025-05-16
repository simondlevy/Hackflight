#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

standard = np.loadtxt('standard.csv', skiprows=3)
haskell = np.loadtxt('haskell.csv', skiprows=3)
plt.plot(standard)
plt.plot(haskell)
plt.legend(('standard', 'haskell'))
plt.show()


