from controller import Keyboard

class Sticks:

    def __init__(self, timestep):

        self.keyboard = Keyboard()

        self.keyboard.enable(timestep)

    def getDemands(self):
        '''
        For now we use only the keyboard
        '''

        t, r, p, y = 0, 0, 0, 0

        key = self.keyboard.getKey()

        if key == Keyboard.UP:
            p = +0.5

        if key == Keyboard.DOWN:
            p = -0.5

        if key == Keyboard.RIGHT:
            r = +0.5

        if key == Keyboard.LEFT:
            r = -0.5

        if key == 81: # Q
            y = -0.5

        if key == 69: # E
            y = +0.5

        if key == 87: # W
            t = +0.5

        if key == 83: # S
            t = -0.5

        return t, r, p, y
