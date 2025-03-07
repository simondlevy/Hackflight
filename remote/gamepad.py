#!/usr/bin/python3

import inputs
from threading import Thread
from time import sleep

def gamepad_threadfun(vals, running):

    while running[0]:

        print(vals)

        sleep(0)  # yield

    
AXIS_MAP = {'X': 0, 'Y': 1, 'Z': 2, 'RX': 3, 'RY': 4, 'RZ': 5}
    
gamepad_vals = [0, 1024, 1024, 1024, 0, 0]

running = [True]

Thread(target=gamepad_threadfun, args=(gamepad_vals, running)).start()

while running[0]:

    print(gamepad_vals)

    try:

        for event in inputs.get_gamepad():

            code = str(event.code)

            if 'ABS' in code:

                axis = AXIS_MAP[code[4:]]

                gamepad_vals[axis] = event.state

    except inputs.UnpluggedError:
        print('No gamepad detected')
        running[0] = False

    except KeyboardInterrupt:
        running[0] = False

    except OSError:
        print('Gamepad unplugged')
        running[0] = False




