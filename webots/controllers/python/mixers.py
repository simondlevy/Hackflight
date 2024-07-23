def runcf(demands):
    '''
    Motor mixer for Crazyflie motor layout
    '''

    t, r, p, y = demands

    m1 = t - r - p  + y  # Front right
    m2 = t - r + p  - y  # Back right
    m3 = t + r + p  + y  # Back left
    m4 = t + r - p  - y  # Front left

    return m1, m2, m3, m4
 

