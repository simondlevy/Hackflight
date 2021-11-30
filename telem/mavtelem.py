from serial import Serial
from sys import stdout

port = Serial('COM69', 115200)

while True:

    try:

        byte = ord(port.read(1))

        if byte == 0xFE:
            print('===============================')

        print('x%02X' % byte)

        stdout.flush()

    except KeyboardInterrupt:

        break

port.close()


