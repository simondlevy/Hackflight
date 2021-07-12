#!/usr/bin/env python3
'''
Hackflight in PythoSimple take-off-and-move-forward scriptn

Copyright (C) 2021 S.Basnet, N. Manaye, N. Nguyen, S.D. Levy

MIT License
'''

import numpy as np
import argparse
from argparse import ArgumentDefaultsHelpFormatter
import cv2
from threading import Thread
import socket
from sys import stdout
from time import sleep

from receiver import Receiver
from mixers import mixer_quadxap, mixer_coaxial
from pidcontrollers import RatePid, YawPid, LevelPid


def _handleImage(image):
    cv2.imshow('Image', image)
    cv2.waitKey(1)


def _debug(msg):
    print(msg)
    stdout.flush()


def _make_udpsocket():

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)

    return sock


def _run_telemetry(host,
                   motorPort,
                   telemetryServerSocket,
                   motorClientSocket,
                   receiver,
                   pid_controllers,
                   mixer,
                   done):

    running = False

    while True:

        try:
            data, _ = telemetryServerSocket.recvfrom(8*13)
        except Exception:
            done[0] = True
            break

        telemetryServerSocket.settimeout(.1)

        telem = np.frombuffer(data)

        # time = telem[0]
        state = telem[1:]

        if not running:
            _debug('Running')
            running = True

        if telem[0] < 0:
            motorClientSocket.close()
            telemetryServerSocket.close()
            break

        # Start with demands from receiver
        demands = np.array(list(receiver.getDemands()))

        # Pass demands through closed-loop controllers
        for pid_controller in pid_controllers:
            demands = pid_controller.modifyDemands(state, demands)

        # Run mixer on demands to get motor values
        motorvals = mixer(demands)

        # Send motor values to client (simulator)
        motorClientSocket.sendto(np.ndarray.tobytes(motorvals),
                                 (host, motorPort))

        # Yield to main thread
        sleep(.001)


def main(host='127.0.0.1',
         motorPort=5000,
         telem_port=5001,
         image_port=5002,
         image_rows=480,
         image_cols=640):

    parser = argparse.ArgumentParser(
            formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('--vehicle', required=False, default='Phantom',
                        help='Vehicle name')

    args = parser.parse_args()

    mixerdict = {'Phantom': mixer_quadxap, 'Ingenuity': mixer_coaxial}

    if args.vehicle not in mixerdict:
        print('Unrecognized vehicle: %s' % args.vehicle)
        exit(1)

    receiver = Receiver()

    mixer = mixerdict[args.vehicle]

    pid_controllers = (RatePid(0.225, 0.001875, 0.375),
                       YawPid(2.0, 0.1),
                       LevelPid(0.2))

    # Allows telemetry thread to tell this thread when user closes socket
    done = [False]

    # Telemetry in and motors out run on their own thread
    motorClientSocket = _make_udpsocket()
    telemetryServerSocket = _make_udpsocket()
    telemetryServerSocket.bind((host, telem_port))

    _debug('Hit the Play button ...')

    # Serve a socket with a maximum of one client
    imageServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    imageServerSocket.bind((host, image_port))
    imageServerSocket.listen(1)

    # This will block (wait) until a client connets
    imageConn, _ = imageServerSocket.accept()
    imageConn.settimeout(1)
    _debug('Got a connection!')

    # Start telemetry thread
    Thread(target=_run_telemetry,
           args=(host,
                 motorPort,
                 telemetryServerSocket,
                 motorClientSocket,
                 receiver,
                 pid_controllers,
                 mixer,
                 done)).start()

    while not done[0]:

        try:
            imgbytes = imageConn.recv(image_rows*image_cols*4)

        except Exception:  # likely a timeout from sim quitting
            break

        if len(imgbytes) == image_rows*image_cols*4:

            rgba_image = np.reshape(np.frombuffer(imgbytes, 'uint8'),
                                    (image_rows, image_cols, 4))

            image = cv2.cvtColor(rgba_image, cv2.COLOR_RGBA2RGB)

            _handleImage(image)

        receiver.update()


main()
