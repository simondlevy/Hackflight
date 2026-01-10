class ZDist:

    ZDIST_INIT = 0.4
    ZDIST_MAX = 1.0
    ZDIST_MIN = 0.2
    ZDIST_INC = 0.01

    def __init__(self):

        self.value = self.ZDIST_INIT

    def update(self, thrust):

        self.thrust = (
                -self.scale(self.gamepad_vals[0]) * self.ZDIST_INC)

        self.zdist = min(max(self.zdist + self.thrust, self.ZDIST_MIN),
                         self.ZDIST_MAX)


