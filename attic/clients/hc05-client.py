#!/usr/bin/python3

import socket
from sys import stdout

import msp

BT_ADDRESS = 'B8:27:EB:3F:AB:47'
BT_PORT = 2


class StateParser(msp.Parser):

    def __init__(self):

        msp.Parser.__init__(self)

    def handle_STATE(self, x, dx, y, dy, z, dz, phi, dphi, theta, dtheta,
                     psi, dpsi):

        print(('dx=%+03.2f dy=%+03.2f z=%+03.2f dz=%+03.2f ' +
               'phi=%+5.1f dphi=%+6.1f theta=%+5.1f dtheta=%+6.1f ' +
               'psi=%+5.1f dpsi=%+5.1f') %
              (dx, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi))


def main():

    print('Connecting to server %s:%d ... ' % (BT_ADDRESS, BT_PORT), end='')
    stdout.flush()

    # Create a Bluetooth or IP socket depending on address format
    client = (socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
              socket.BTPROTO_RFCOMM))

    try:
        client.connect((BT_ADDRESS, BT_PORT))

    except Exception as e:
        print(str(e) + ': is server running?')
        exit(0)

    byte = None

    parser = StateParser()

    count = 0

    while True:

        try:

            msg = client.recv(48)

            print(count)

            count += 1

            # byte = client.recv(1)

        except KeyboardInterrupt:
            break

        # if byte is not None:
        #    parser.parse(byte)


main()
