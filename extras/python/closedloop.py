"""
Abstract class for closed loop controller in python

Copyright (C) 2021 S.Basnet, N. Manaye, N. Nguyen, S.D. Levy

MIT License
"""


import abc


class ClosedLoopController(object, metaclass=abc.ABCMeta):

    @abc.abstractmethod
    def modifyDemands(state, demands):
        """
        Retruns updated demands based on current demands and b=vehicle state
        """
        return
