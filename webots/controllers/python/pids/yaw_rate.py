from . pid import PID


class YawRateController:

    '''
    Demand is input in degrees per second and output in units
    appropriate for our motors both nose-right positive.                                                       
    '''

    def __init__(self):

        self.pid = PID()

    def run(self, kp, dpsi, dt, demands):

        t, r, p, yaw = demands

        yaw = self.pid.run_p(kp, dt, yaw, dpsi)

        return t, r, p, yaw
