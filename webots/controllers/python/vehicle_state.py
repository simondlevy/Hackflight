import numpy as np

class Axis3:

    def __init__(self):

        self.x = 0
        self.y = 0
        self.z = 0

class VehicleState:

    def __init__(self):

        self.ang = Axis3()
        self.dang = Axis3()
        self.pos = Axis3()
        self.dpos = Axis3()

        self.tprev = 0

        self.xprev = 0
        self.yprev = 0
        self.zprev = 0

    def __str__(self):

        fmt = ('x=%+3.3f dx=%+3.3f ' + 
               'y=%+3.3f dy=%+3.3f ' +
               'z=%+3.3f dz=%+3.3f ' +
               'phi=+%3.3f dphi=%+3.3f ' + 
               'theta=%+3.3f dtheta=%+3.3f ' + 
               'psi=%+3.3f dpsi=%+3.3f')
                
        return fmt % (
                self.pos.x, self.dpos.x, 
                self.pos.y, self.pos.y, 
                self.pos.z, self.pos.z,
                self.ang.x, self.dang.x,
                self.ang.y, self.ang.y,
                self.ang.z, self.dang.z)

    def get(self, robot, gyro, imu, gps):

        # Get deltaT
        tcurr = robot.getTime()
        dt =  tcurr - self.tprev
        self.tprev = tcurr

        # Get yaw angle in radians
        psi = imu.getRollPitchYaw()[2]

        # Get state variables, negating gyro for nose-right positive
        self.pos.x = gps.getValues()[0]
        self.pos.y = gps.getValues()[1]
        self.pos.z = gps.getValues()[2]
        self.ang.x = np.degrees(imu.getRollPitchYaw()[0])
        self.dang.x =  np.degrees(gyro.getValues()[0])
        self.ang.y = np.degrees(imu.getRollPitchYaw()[1])
        self.dang.y =  np.degrees(gyro.getValues()[1])
        self.ang.z  =  -np.degrees(psi) 
        self.dang.z =  -np.degrees(gyro.getValues()[2])

        # Use temporal first difference to get world-cooredinate velocities
        x = gps.getValues()[0]
        y = gps.getValues()[1]
        dx = (x - self.xprev) / dt
        dy = (y - self.yprev) / dt
        self.dpos.z = (self.pos.z - self.zprev) / dt

        # Rotate X,Y world velocities into body frame to simulate optical-flow
        # sensor
        cospsi = np.cos(psi)
        sinpsi = np.sin(psi)
        self.dpos.x = dx * cospsi + dy * sinpsi
        self.dpos.y = dx * sinpsi - dy * cospsi

        # Save past time and position for next time step
        self.xprev = x
        self.yprev = y
        self.zprev = self.pos.z
