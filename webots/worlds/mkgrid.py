#!/usr/bin/python3

import numpy as np

N = 34
H = 0.2

a = H * np.random.random((N, N))

for j in range(N):
    for k in range(N):
        print('%3.2f' % a[j][k], end=',')
    print()
