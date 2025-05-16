#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('climbsnn.csv', delimiter=',', skiprows=2)

snn = data[:,0]
pid = data[:,1]

m,b = np.polyfit(snn, pid, 1)

fit = snn * m + b

plt.plot(pid)
plt.plot(snn)
plt.plot(fit)

model = 'm = %f  b = %f' % (m, b)

print(model)

plt.title(model)

plt.legend(('PID', 'SNN', 'FIT'))

plt.show()
