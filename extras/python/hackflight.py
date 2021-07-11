#!/usr/bin/env python3
'''
Hackflight in PythoSimple take-off-and-move-forward scriptn

Copyright (C) 2021 S.Basnet, N. Manaye, N. Nguyen, S.D. Levy

MIT License
'''

import numpy as np
import argparse
from argparse import ArgumentDefaultsHelpFormatter

from multicopter_server import MulticopterServer

from receiver import Receiver
from mixers import QuadXAPMixer, CoaxialMixer
from pidcontrollers import RatePid, YawPid, LevelPid

from debugging import debug


class HackflightCopter(MulticopterServer):

    def __init__(self, receiver, mixer, pid_controllers):

        MulticopterServer.__init__(self)

        self.receiver = receiver
        self.mixer = mixer
        self.pid_controllers = pid_controllers

        self.sensors = []
        self.closedloops = []

    def begin(self):

        self.receiver.begin()

        MulticopterServer.start(self)

    def getMotors(self, t, state):

        # Start with demands from receiver
        demands = np.array(list(self.receiver.getDemands()))

        # Pass demands through closed-loop controllers
        for pid_controller in self.pid_controllers:
            demands = pid_controller.modifyDemands(state, demands)

        motors = self.mixer.getMotors(demands)

        return motors

    def handleImage(self, image):
        '''
        Since this method runs on the main thread, we can call it to update the
        receiver '''

        self.receiver.update()

    def addSensor(self, sensor):

        self.sensors.append(sensor)


def main():

    parser = argparse.ArgumentParser(
            formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('--vehicle', required=False, default='Phantom',
                        help='Vehicle name')

    args = parser.parse_args()

    mixerdict = {'Phantom': QuadXAPMixer, 'Ingenuity': CoaxialMixer}

    if args.vehicle not in mixerdict:
        print('Unrecognized vehicle: %s' % args.vehicle)
        exit(1)

    # Create Hackflight object
    h = HackflightCopter(Receiver(),
                         mixerdict[args.vehicle](),
                         (RatePid(0.225, 0.001875, 0.375),
                          YawPid(2.0, 0.1),
                          LevelPid(0.2)))

    # Go!
    h.begin()


main()
