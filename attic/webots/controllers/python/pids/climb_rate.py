from . pid import PID


class ClimbRateController:
    '''
    Demand is input as climb rate in meters per second and output as positive
    value scaled according to motor characteristics.
     '''

    KP = 25
    KI = 0 # 15
    ILIMIT = 5000

    def __init__(self):

        self.pid = PID()

    def run(self, dz, dt, tbase, tscale, tmin, flying, demands):

        thrust, r, p, y = demands

        thrustpid = self.pid.run_pi(
                self.KP, self.KI, self.ILIMIT, dt, thrust, dz)

        thrust = thrustpid * tscale + tbase if flying else tmin

        return thrust, r, p, y
