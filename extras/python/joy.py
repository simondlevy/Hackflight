import pygame as pg
from debugging import debug

def loop():

    while True:

        pg.event.get()

        joystick = pg.joystick.Joystick(0)
        joystick.init()

        debug(list(joystick.get_axis(i) for i in range(joystick.get_numaxes())))


def main():

    pg.init()
    pg.joystick.init()

    loop()

main()
