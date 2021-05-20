"""
Test program for python version of hackflight

Copyright (C) 2021 S.Basnet, N. Manaye, N. Nguyen, S.D. Levy

MIT License
"""


from hackflight import Hackflight
from board import Board
from receiver import Receiver
from passthru_controller import PassThruController
# from quadxap_mixer import QuadXAPMixer
from newQuadXAPMixer import Mixer
from debugging import debug


def main():

    board = Board()
    receiver = Receiver()
    # actuator = QuadXAPMixer(None)
    actuator = Mixer()

    h = Hackflight(board, receiver, actuator)
    h.addClosedLoopController(PassThruController())

    h.begin()

    while True:

        try:
            demands = h.update()
            # debug(demands)

            omega = actuator.run(demands)
            """
        3cw   1ccw
           |  /
            ^
          /   |
        2ccw  4cw
            """
            debug("1: %+3.3f 2: %+3.3f 3: %+3.3f 4: %+3.3f " %
                  tuple(omega))

        # Exit gracefully on CRTL-c
        except KeyboardInterrupt:
            break


main()
