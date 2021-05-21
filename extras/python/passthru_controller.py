"""
Pass through closed loop controller

Allows us to pass the open loop demands directly to the mixer

Copyright (C) 2021 S.Basnet, N. Manaye, N. Nguyen, S.D. Levy

MIT License
"""


from closedloop import ClosedLoopController


class PassThruController(ClosedLoopController):

    def modifyDemands(state, demands):
        return demands
