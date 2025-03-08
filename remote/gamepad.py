#!/usr/bin/python3

import inputs
from threading import Thread
from time import sleep

def gamepad_threadfun(vals, status):

    while status['running']:

        scaled = tuple(map(lambda x : x/32768,
                          (-vals[0], vals[1], -vals[2], vals[3])))

        print('t=%+3.3f  r=%+3.3f  p=%+3.3f  y=%+3.3f' % scaled, end=' ')

        print('Armed: %d' % status['armed'])

        sleep(0)  # yield


def main():

    SUPPORTED = { 'Microsoft X-Box 360 pad',  'Logitech Gamepad F310' }

    AXIS_MAP = {'X': 3, 'Y': 0, 'Z': 1, 'RX': 1, 'RY': 2, 'RZ': 2}

    gamepads = inputs.devices.gamepads

    if len(gamepads) == 0:
        print('No gamepad detected')
        exit(0)
        
    devname = inputs.devices.gamepads[0].name 

    if not devname in SUPPORTED:
        print(devname + ' not supported')
        exit(0)

    gamepad_vals = [0, 0, 0, 0]

    status = {'running': True, 'armed': False}

    button_state_prev = 0

    Thread(target=gamepad_threadfun, args=(gamepad_vals, status)).start()

    while status['running']:

        try:

            for event in inputs.get_gamepad():

                code = str(event.code)

                if 'ABS' in code:

                    subcode = code[4:]

                    if subcode in AXIS_MAP:

                        axis = AXIS_MAP[subcode]

                        gamepad_vals[axis] = event.state

                elif code in {'BTN_TR', 'BTN_PINKIE'} :

                    if not event.state and button_state_prev:

                        status['armed'] = not status['armed']

                    button_state_prev = event.state

        except inputs.UnpluggedError:
            print('No gamepad detected')
            status['running'] = False

        except KeyboardInterrupt:
            status['running'] = False

        except OSError:
            print('Gamepad unplugged')
            status['running'] = False


main()
