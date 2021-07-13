import pygame as pg
from debugging import debug

def init():

    pg.init()
    pg.joystick.init()

    js = pg.joystick.Joystick(0)
    js.init()

    return js

def loop(js):

    while True:

        pg.event.get()

        debug(js.get_axis(1))

def main():

    js = init()

    loop(js)

main()
