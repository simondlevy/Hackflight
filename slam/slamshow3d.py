#!/usr/bin/python

'''
Adapted from http://2a42.github.io/code/voxel.py
'''

import pyglet
import pyglet.gl as pgl
import math 
import threading
import time

class _VoxelEngine:

    def __init__(self):

        # Create list for sparse matrix
        self.occupied = []

    def set(self, x, y, z):
        
        self.occupied.append((x,y,z))

    def draw(self):

        vertices = (
            0, 0, 0,    # vertex 0
            0, 0, 1,    # vertex 1
            0, 1, 0,    # vertex 2
            0, 1, 1,    # vertex 3
            1, 0, 0,    # vertex 4
            1, 0, 1,    # vertex 5
            1, 1, 0,    # vertex 6
            1, 1, 1,    # vertex 7
        )

        faces = (
            0, 1, 3, 2,    # top face
            4, 5, 7, 6,    # bottom face
            0, 4, 6, 2,    # left face
            1, 5, 7, 3,    # right face
            0, 1, 5, 4,    # down face
            2, 3, 7, 6,    # up face
        )

        # Loop through occupied voxels
        for voxel in self.occupied:

            x,y,z = voxel[0], voxel[1], voxel[2]

            pgl.glTranslated(x, y, z)

            # Draw voxel in gray
            pgl.glPolygonMode(pgl.GL_FRONT_AND_BACK, pgl.GL_FILL)
            pgl.glColor3ub(127,127,127)
            pyglet.graphics.draw_indexed(8, pgl.GL_QUADS, faces, ('v3i', vertices))

            # Draw edges in black
            pgl.glPolygonMode(pgl.GL_FRONT_AND_BACK, pgl.GL_LINE)
            pgl.glColor3ub(0, 0, 0)
            pyglet.graphics.draw_indexed(8, pgl.GL_QUADS, faces, ('v3i', vertices))

            pgl.glTranslated(-x, -y, -z)


class _Window(pyglet.window.Window):

    def __init__(self, engine, size):

        super(_Window, self).__init__(size, size, resizable=True, caption='3D SLAM Visualization')

        pgl.glClearColor(0, 0, 0, 1)

        # Start at a reasonable camera distance
        self.viewerDistance = 30

        # Start at a pleasing camera angle (pan,tilt=45 degrees)
        self.pan  = math.pi / 4
        self.tilt = math.pi / 4

        # Start at neutral vertical reference
        self.vref = 0

        # Keep a handle to the voxel engine
        self.engine = engine

        # Keep a flag for when we close the window, to stop the thread
        self.running = True

    def update_angle(self, angle, diff, size, maxangle):

        angle += diff / float(size)  * 2 * math.pi/2

        return min(max(angle, -maxangle), maxangle)

    def on_mouse_drag(self, x, y, dx, dy, button, modifiers):

        # Right button for pan/tilt
        if button == pyglet.window.mouse.RIGHT:
            self.pan  = self.update_angle(self.pan,   dx, self.width, math.pi)
            self.tilt = self.update_angle(self.tilt, -dy, self.height, math.pi/2)

        # Left button for strafe
        elif button == pyglet.window.mouse.LEFT:

            self.vref -= dy/10.

    def on_mouse_scroll(self, x, y, scroll_x, scroll_y):

        self.viewerDistance -= scroll_y

    def on_draw(self):

        self.clear()

        # Setup the 3D matrix
        pgl.glMatrixMode(pgl.GL_PROJECTION)
        pgl.glLoadIdentity()
        pgl.gluPerspective(self.viewerDistance, self.width / float(self.height), 0.1, 200)
        pgl.glMatrixMode(pgl.GL_MODELVIEW)
        pgl.glLoadIdentity()

        # Compute "eye" position based on pan and tilt angles
        eyex = int(math.cos(self.pan)  * 50)
        eyey = int(math.sin(self.tilt) * 100)
        eyez = int(math.sin(self.pan)  * 50)

        centerx = 0 
        centery = int(self.vref)
        centerz = 0

        pgl.gluLookAt(eyex, eyey, eyez, 
                centerx, centery, centerz,
                0, 1, 0)    # Up vector

        self.engine.draw()

    def on_window_close(self):

        self.running = False

# Public class -----------------------------------------------------------------------

class SlamShow3D(object):

    def __init__(self, resolution, canvasSize=1000):

        # The model
        self.engine = _VoxelEngine()
        self.res = resolution

        # The view
        self.window = _Window(self.engine, canvasSize)
        self.canvsize = canvasSize

    def run(self):

        # Schedule a clock event to display voxels
        pyglet.clock.schedule_interval(self._update, 0.01)

        # Start user interaction
        pyglet.app.run()

    def addObstacle(self, x, y, z):

        # Scale the point to the resolution
        x, y, z = tuple([int(v/float(self.res)*self.canvsize) for v in (x,y,z)])

        self.engine.set(x, y,z)

    def _update(self, dt):

        self.window.on_draw()

# Test -------------------------------------------------------------------------------

def _listener(slamvis):

    x = 0
    z = 0
    y = 10
    b = 0
    c = 0
    while slamvis.window.running:


        if z < 10:
            slamvis.addObstacle(x+10,y,z)
            slamvis.addObstacle(x+4, y, z)
            z += 1   

            if z == 10:
                z  = 0
                y += 1
                b += 1
                if b > 4:
                    y = 10
                    break
                    
    while slamvis.window.running:
       

        if x < 7:
            slamvis.addObstacle(x+4,y,4)
            x+=1

            if x== 7:
                x = 0
                y += 1
                c += 1
                if c > 4:       
                    break

        time.sleep(.1)

if __name__ == '__main__':

    slamvis = SlamShow3D(800, 800) # same resolution as canvas size

    thread = threading.Thread(target=_listener, args=(slamvis,))
    thread.daemon = True
    thread.start()

    slamvis.run()
