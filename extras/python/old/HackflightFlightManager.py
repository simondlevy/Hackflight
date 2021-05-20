"""
"""

#all this importation stuff?

import SimReceiver
import SimBoard
import SimMotor
import SimSensors

class FHackflightFlightManager(FFlightManager):

    def __init__(self, pawn, mixer, motors, dynamics\
        pidEnabled = True ):

        #???
        FFlightManager(dynamics)

        self._motors = motors
        #Unreal engine function?
        self._receiver = SimReceiver(#something, 0)
        self._hackflight = Hackflight(_board, _receiver, mixer)

        self._sensors = SimSensors(_dynamics)
        self._hackflight.addSensor(_sensors)

        if pidEnabled:
            self._hackflight.addClosedLoopController(levelPid)
            self._hackflight.addClosedLoopController(ratePid)
            self._hackflight.addClosedLoopController(yawPid)

        self._hackflight.begin(True)
    
    #override?
    def getMotors(self, time, motorvals):

        joystickError = self._receiver.update()

        angularVel = [#???]

        eulerAngles = [#???]

        quaternion = [#???]

        #WHat's this for?
        #if joystickError:

        self._hackflight.update()
        self._board.set(time)

        for i in range (self._nmotors):
            motorvals[i] = self._motors.getValue(i)

    def tick(self, None):
        self._receiver.tick()


