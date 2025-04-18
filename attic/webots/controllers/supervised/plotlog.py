#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

time = data[:,0]

# Motors ------------------------------------

plt.subplot(4, 1, 1)

plt.plot(time, data[:,1], '-+')  # m1
plt.plot(time, data[:,2])        # m2
plt.plot(time, data[:,3], '-o')  # m3
plt.plot(time, data[:,4])        # m4

mid = 55.375
mar = 1.6

plt.plot(time, mid * np.ones(time.shape))

#plt.ylim([mid-mar, mid+mar])

plt.legend(['m1', 'm2', 'm3', 'm4'])

plt.ylabel('motors (rad / sec)')

# U2 -----------------------------------------

plt.subplot(4, 1, 2)
plt.ylabel('U2')
plt.plot(time, data[:,5])

# Roll rate ----------------------------------

rng = 50

plt.subplot(4, 1, 3)
plt.plot(time, data[:,6])
plt.ylim((-rng, +rng))
plt.ylabel('dphi/dt (deg / sec)')

# Roll angle ----------------------------------

rng = 10

plt.subplot(4, 1, 4)
plt.plot(time, data[:,7])
plt.ylim((-rng, +rng))
plt.ylabel('phi (deg)')

plt.xlabel('time (sec)')

plt.show()

