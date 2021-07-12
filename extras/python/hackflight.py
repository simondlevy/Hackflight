#!/usr/bin/env python3
'''
Hackflight in PythoSimple take-off-and-move-forward scriptn

Copyright (C) 2021 S.Basnet, N. Manaye, N. Nguyen, S.D. Levy

MIT License
'''

import numpy as np
import argparse
from argparse import ArgumentDefaultsHelpFormatter

from server import start

from receiver import Receiver
from mixers import QuadXAPMixer, CoaxialMixer
from pidcontrollers import RatePid, YawPid, LevelPid


def _updateReceiver(parts):

    receiver, _mixer, _pid_controllers = parts

    receiver.update()


def _getMotors(parts, time, state):

    receiver, mixer, pid_controllers = parts

    # Start with demands from receiver
    demands = np.array(list(receiver.getDemands()))

    # Pass demands through closed-loop controllers
    for pid_controller in pid_controllers:
        demands = pid_controller.modifyDemands(state, demands)

    motors = mixer.getMotors(demands)

    return motors


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

    receiver = Receiver()
    receiver.begin()

    mixer = mixerdict[args.vehicle]()

    pid_controllers = (RatePid(0.225, 0.001875, 0.375), YawPid(2.0, 0.1), LevelPid(0.2))

    start((receiver, mixer, pid_controllers), _updateReceiver, _getMotors)


main()
