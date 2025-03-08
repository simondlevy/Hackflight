#!/usr/bin/python3

import inputs
from threading import Thread
from time import sleep

def gamepad_threadfun(vals, status):

    while status['running']:

        print(vals)

        sleep(0)  # yield


def main():
    
    AXIS_MAP = {'X': 3, 'Y': 0, 'Z': 1, 'RX': 1, 'RY': 2, 'RZ': 2}
        
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
