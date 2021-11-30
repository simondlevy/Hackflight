from serial import Serial
from sys import stdout

port = Serial('COM69', 115200)

while True:

    try:

        print('x%02X' % ord(port.read(1)))
        stdout.flush()

    except KeyboardInterrupt:

        break

port.close()


