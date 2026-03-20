#!/usr/bin/python3

import numpy as np

a = np.arange(0, 49).reshape((7,7)).astype(np.float32)

b = np.arange(50,99).reshape((7,7)).astype(np.float32)

print(np.dot(a,b))
