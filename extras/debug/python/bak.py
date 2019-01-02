import serial
from realtime_plot import RealtimePlotter
from threading import Thread
from time import time

'''

class SerialPlotter(RealtimePlotter):

    def __init__(self):

        RealtimePlotter.__init__(self, [(-1,+1), (-1,+1)], 
                phaselims=((-1,+1), (-1,+1)),
                window_name='Position',
                yticks = [(-1,0,+1),(-1,0,+1)],
                styles = ['r--', 'b-'], 
                ylabels=['X', 'Y'])

        
        self.xcurr = 0
        self.start_time = time()
        self.start_pos = None
        self.pos = (0,0)


    def getValues(self):

         return self.pos[0], self.pos[1], self.pos[0], self.pos[1]

def _update(port, plotter):

    msg = ''

    while True:

        c = port.read().decode()

        if c == '\n':
            
            try:

                pos = tuple((float(v) for v in msg.split()))
                
            except:
                
                pass

            if plotter.start_pos is None:

                if (time() - plotter.start_time) > DELAY:

                    plotter.start_pos = pos
            else:

                plotter.pos = pos[0]-plotter.start_pos[0], pos[1]-plotter.start_pos[1]
            
                #print('%+3.3f %+3.3f' % (plotter.pos[0], plotter.pos[1]))
                print('%+3.3f %+3.3f' % (pos[0], pos[1]))
            
            msg = ''
            
        else:
            
            msg += c

        plotter.xcurr += 1
'''

    '''
    try:
        port = serial.Serial(PORT, BAUD)
    except serial.SerialException:
        print('Unable to access device on port %s' % PORT)
        exit(1)

    plotter = SerialPlotter()

    thread = Thread(target=_update, args = (port, plotter))
    thread.daemon = True
    thread.start()

    plotter.start()
    '''
