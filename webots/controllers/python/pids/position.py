from . pid import PID


class PositionController:
    '''
     Demand is input as desired speed in meter per second, output as
     angles in degrees.
    '''

    KP = 25

    def __init__(self):

        self.roll_pid = PID()

        self.pitch_pid = PID()

    def run(self, dx, dy, dt, demands):

        thrust, roll, pitch, yaw = demands

        roll = self.run_axis(self.roll_pid, roll, dt, dy)

        pitch = self.run_axis(self.pitch_pid, pitch, dt, dx)

        return thrust, roll, pitch, yaw

    def run_axis(self, pid, demand, dt, actual):

        return pid.run_p(self.KP, dt, demand, actual)
