#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

mphi = data[:, 0]
mdphi = data[:, 1]
mtheta = data[:, 2]
mdtheta = data[:, 3]
mpsi = data[:, 4]
mdpsi = data[:, 5]

ephi = data[:, 6]
edphi = data[:, 7]
etheta = data[:, 8]
edtheta = data[:, 9]
epsi = data[:, 10]
edpsi = data[:, 11]

plt.subplot(6, 1, 1)
plt.plot(mphi)
plt.plot(ephi)
plt.ylabel('phi')
plt.legend(('Madg',  'EKF'))

plt.subplot(6, 1, 2)
plt.plot(mtheta)
plt.plot(etheta)
plt.ylabel('theta')
plt.legend(('Madg', 'EKF'))

plt.show()
