import numpy as np

def get_vehicle(width, depth, length):

    points = np.array([
            [-width, +depth, -length],
            [+width, +depth, -length],
            [+width, -depth, -length],
            [-width, -depth, -length],
            [-width, +depth, +length],
            [+width, +depth, +length],
            [+width, -depth, +length],
            [-width, -depth, +length]
            ])

    # Each face contains indices into points array above
    faces = [(0,1,2,3),(1,5,6,2),(5,4,7,6),(4,0,3,7),(0,4,5,1),(3,2,6,7)] 

    colors = ['red', 'green', 'blue', 'yellow', 'cyan', 'white']

    return points, faces, colors
