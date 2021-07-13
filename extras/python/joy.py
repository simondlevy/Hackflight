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

        debug(list(joystick.get_axis(i) for i in range(joystick.get_numaxes())))


def main():

    init()

    loop()

main()
