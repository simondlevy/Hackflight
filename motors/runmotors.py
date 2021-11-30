from mspparser import MspParser
from serial import Serial
from time import sleep
from sys import stdout

class MyParser(MspParser):

    def handle_ATTITUDE(self, angx, angy, heading):
        print(angx, angy, heading)

PORT = 'COM31'

port = Serial(PORT, 115200, timeout=1)

cmd = MspParser.serialize_ATTITUDE_Request()

parser = MyParser()

port.write(cmd)

while True:

    try:
        byte = port.read(1)
        print(byte)
        stdout.flush()
        sleep(.001)

    except KeyboardInterrupt:
        break
