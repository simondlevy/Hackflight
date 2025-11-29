#!/usr/bin/python3

import numpy as np

N = 6
H = 0.1

a = H * np.random.random((N, N))

for j in range(N):
    for k in range(N):
        print('%3.2f' % a[j][k], end=',')
    print()
