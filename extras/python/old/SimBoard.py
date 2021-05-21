"""
Authors: Sujana, Nobel
Hackflight Board Class implementation for MulticopterSim.
Pyhton translation.
"""

from RFT_board import Board

class SimBoard(Board):
    """A Board Class implementation."""
    def __init__(self):
        self._currentTime = 0.0

    def _getTime(self):
        return self._currentTime
        
    def set(self, time):
        self._currentTime = time

#Do we need a destructor method?
