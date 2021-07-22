#!/usr/bin/env python3
'''
Hackflight in Python

Uses MulticopterSim SocketModule

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

from mixers import mixer_quadxap, mixer_coaxial
from pidcontrollers import (rate_pid, level_pid, yaw_pid, alt_hold_pid,
                            pos_hold_pid)


def _handleImage(image):
    cv2.imshow('Image', image)
    cv2.waitKey(1)


def _debug(msg):
    print(msg)
    stdout.flush()


def _udpsocket():

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)

    return sock


def _run_telemetry(host, motor_port, telemetry_server_socket,
                   motor_client_socket, pid_controllers, mixer, done):

    running = False

    pid_states = [p[1] for p in pid_controllers]

    while True:

        try:
            data, _ = telemetry_server_socket.recvfrom(8*17)
        except Exception:
            done[0] = True
            break

        telemetry_server_socket.settimeout(.1)

        telem = np.frombuffer(data)

        # time = telem[0]
        vehicle_state = telem[1:13]

        if not running:
            _debug('Running')
            running = True

        if telem[0] < 0:
            motor_client_socket.close()
            telemetry_server_socket.close()
            break

        # Start with demands from controller
        demands = telem[13:]

        # Pass demands through closed-loop controllers
        for k, p in enumerate(pid_controllers):
            demands, pid_states[k] = p[0](vehicle_state, pid_states[k],
                                          demands)

        # Run mixer on demands to get motor values
        motorvals = mixer(demands)

        # Send motor values to client (simulator)
        motor_client_socket.sendto(np.ndarray.tobytes(motorvals),
                                   (host, motor_port))

        # Yield to main thread
        sleep(.001)


def main(host='127.0.0.1',
         motor_port=5000,
         telem_port=5001,
         image_port=5002,
         image_rows=480,
         image_cols=640):

    parser = argparse.ArgumentParser(
            formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('--vehicle', required=False, default='Phantom',
                        help='Vehicle name')

    args = parser.parse_args()

    mixer_dict = {'Phantom': mixer_quadxap, 'Ingenuity': mixer_coaxial}

    if args.vehicle not in mixer_dict:
        print('Unrecognized vehicle: %s' % args.vehicle)
        exit(1)

    mixer = mixer_dict[args.vehicle]

    pid_controllers = (# pos_hold_pid(),
                       rate_pid(),
                       yaw_pid(),
                       # level_pid(),
                       # alt_hold_pid()
                       )

    # Allows telemetry thread to tell this thread when user closes socket
    done = [False]

    # Telemetry in and motors out run on their own thread
    motor_client_socket = _udpsocket()
    telemetry_server_socket = _udpsocket()
    telemetry_server_socket.bind((host, telem_port))

    _debug('Hit the Play button ...')

    # Serve a socket with a maximum of one client
    image_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    image_server_socket.bind((host, image_port))
    image_server_socket.listen(1)

    # This will block (wait) until a client connets
    image_conn, _ = image_server_socket.accept()
    image_conn.settimeout(1)
    _debug('Got a connection!')

    # Start telemetry thread
    Thread(target=_run_telemetry,
           args=(host, motor_port, telemetry_server_socket,
                 motor_client_socket, pid_controllers, mixer,
                 done)).start()

    while not done[0]:

        try:
            imgbytes = image_conn.recv(image_rows*image_cols*4)

        except Exception:  # likely a timeout from sim quitting
            break

        if len(imgbytes) == image_rows*image_cols*4:

            rgba_image = np.reshape(np.frombuffer(imgbytes, 'uint8'),
                                    (image_rows, image_cols, 4))

            image = cv2.cvtColor(rgba_image, cv2.COLOR_RGBA2RGB)

            _handleImage(image)


main()
