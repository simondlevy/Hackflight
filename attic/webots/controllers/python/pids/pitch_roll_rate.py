from . pid import PID


class PitchRollRateController:
    '''
       Demands are input as angular velocities in degrees per second and output
       in uints appropriate for our motors.
     '''
    def __init__(self):

        self.roll = PID()

        self.pitch = PID()

    def run(self, kp, kd, dphi, dtheta, dt, reset, demands):

        t, roll, pitch, y = demands

        nothrust = (t == 0)

        roll = self.run_axis(kp, kd, self.roll, roll, dt, dphi, nothrust)

        pitch = self.run_axis(kp, kd, self.pitch, pitch, dt, dtheta, nothrust)

        return t, roll, pitch, y

    def run_axis(self, kp, kd, pid, demand, dt, actual, reset):

        return pid.run_pd(kp, kd, dt, demand, actual, reset)
