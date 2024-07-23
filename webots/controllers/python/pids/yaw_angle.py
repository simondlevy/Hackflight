from . pid import PID


class YawAngleController():
                                                                   
    '''
    Demand is input as desired angle normalized to [-1,+1] and output        
    as degrees per second, both nose-right positive.                         
    '''

    KP = 6
    KD = 0.25
    ANGLE_MAX = 200

    def __init__(self):

        self.pid =PID()
        self.target = 0

    def run(self, psi, dt, demands):

        t, r, p, yaw = demands

        target = self.cap(self.target + self.ANGLE_MAX * yaw * dt)

        yaw = self.pid.run_pd(self.KP, self.KD, dt, target, psi)

        # Reset target on zero thrust                                      
        self.target = psi if t == 0 else target
        
        return t, r, p, yaw
    
    def cap(self, angle):

        angle1 = angle - 360 if angle > 180 else angle

        return angle1 + 360 if angle1 < (-180) else angle1
