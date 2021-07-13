import pygame as pg
from debugging import debug

def init():

    pg.init()
    pg.joystick.init()


def loop():

    joystick = pg.joystick.Joystick(0)
    joystick.init()

    while True:

        pg.event.get()

        debug(joystick.get_axis(1))


def main():

    init()

    loop()

main()
