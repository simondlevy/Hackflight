from . pid import PID


class AltitudeController:
    '''
    Demand is input as normalized altitude target in meters and output as climb
    rate in meters-per-second
    '''

    KP = 2.0
    KI = 0.5
    ILIMIT = 5000

    def __init__(self):

        self.pid = PID()

    def run(self, z, dt, target, demands):

        t, r, p, y = demands

        t = self.pid.run_pi(self.KP, self.KI, self.ILIMIT, dt, target, z)

        return t, r, p, y
