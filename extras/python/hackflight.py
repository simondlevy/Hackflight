"""
Hackflight class in python

Copyright (C) 2021 S.Basnet, N. Manaye, N. Nguyen, S.D. Levy

MIT License
"""


import numpy as np
# from debugging import debug


class Hackflight(object):

    def __init__(self, board, receiver, actuator):
        self.board = board
        self.receiver = receiver
        self.actuator = actuator

        self.sensors = []
        self.closedloops = []

    def addSensor(self, sensor):
        self.sensors.append(sensor)

    def addClosedLoopController(self, controller):
        self.closedloops.append(controller)

    def begin(self):
        """Set up 12 state values, and initializes other classes
         like board and receiver"""
        # See Bouabdallah (2004)
        self.state = np.zeros(12)

        # Start the board
        self.board.begin()

        # Initialize the sensors
        self._startSensors()

        # Initialize the receiver
        self.receiver.begin()

    def update(self):
        """Get demands from the receiver and state values from
        the sensors"""
        # Grab control signal if available and
        # Run closed loop controllers
        demands = self._runClosedLoop(self.receiver.getDemands())

        # debug("T: %+3.3f R: %+3.3f P: %+3.3f Y: %+3.3f " %
        #       tuple(demands))

        # Check Sensors
        self._checkSensors()

        return demands

    def _startSensors(self):
        return

    def _checkReceiver(self):
        """Run a receiver class method to get the
        data values from the controller."""

        self.receiver.getData()

    def _checkSensors(self):
        return

    def _runClosedLoop(self, demands):

        # debug("T: %+3.3f R: %+3.3f P: %+3.3f Y: %+3.3f " %
        #       tuple(demands))

        for clc in self.closedloops:
            demands = clc.modifyDemands(demands)

        return demands
