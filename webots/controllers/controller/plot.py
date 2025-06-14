#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
from sys import argv

START = 2000

def mklegend(vals):
    return 'min=%d max=%d avg=%d' % (min(vals), max(vals), int(np.mean(vals)))

fname = argv[1]

times = np.loadtxt(fname, delimiter=',')

inp1 = times[START:,0]
inp2 = times[START:,1]
out = times[START:,2]

plt.title(fname)
plt.subplot(2, 1, 1)
plt.plot(inp1)
plt.plot(inp2)
plt.legend([mklegend(inp1), mklegend(inp2)])
plt.subplot(2, 1, 2)
plt.plot(out)
plt.legend([mklegend(out)])
plt.show()

