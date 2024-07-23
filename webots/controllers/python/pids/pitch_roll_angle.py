from . pid import PID


class PitchRollAngleController:
    '''
       Demand is input as angles in degrees and output as angular velocities in
       degrees per second; roll-right / pitch-forward positive.
    '''

    def __init__(self):

        self.roll_pid = PID()

        self.pitch_pid = PID()

    def run(self, kp, phi, theta, dt, demands):

        t, roll, pitch, y = demands

        roll = self.run_axis(kp, self.roll_pid, roll, dt, phi)

        pitch = self.run_axis(kp, self.pitch_pid, pitch, dt, theta)

        return t, roll, pitch, y

    def run_axis(self, kp, pid, demand, dt, actual):

        return pid.run_p(kp, dt, demand, actual)
