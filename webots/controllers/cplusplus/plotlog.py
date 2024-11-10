#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

#fprintf(logfp, "%f,%f,%f,%f\n", z, state.z, dz, state.dz);

plt.subplot(2, 1, 1)
plt.plot(data[:,0], '-+')
plt.plot(data[:,1])
plt.legend(('est', 'true'))
plt.ylabel('z (m)')

plt.subplot(2, 1, 2)
plt.plot(data[:,2], '-+')
plt.plot(data[:,3])
plt.legend(('est', 'true'))
plt.ylabel('dz/dt (m/s)')

plt.show()

