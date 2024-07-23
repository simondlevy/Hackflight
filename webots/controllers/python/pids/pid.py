import numpy as np


class PID:

    def __init__(self):

        self.error = 0
        self.integ = 0

    def _run(self, kp, ki, kd, ilimit, dt, target, actual, reset):

        error = target - actual

        deriv = (error - self.error) / dt

        self.error = 0 if reset else error

        self.integ = (0 if reset
                      else np.clip(self.integ + error * dt, -ilimit, +ilimit))

        return kp * error + kd * deriv

    def run_p(self, kp, dt, target, actual):

        return self._run(kp, 0, 0, 0, dt, target, actual, False)

    def run_pi(self, kp, ki, ilimit, dt, target, actual, reset=False):

        return self._run(kp, ki, 0, ilimit, dt, target, actual, reset)

    def run_pd(self, kp, kd, dt, target, actual, reset=False):

        return self._run(kp, 0, kd, 0, dt, target, actual, reset)
