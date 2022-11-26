'''
This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Hackflight. If not, see <https://www.gnu.org/licenses/>.
'''


import struct

import abc


class MspParser(metaclass=abc.ABCMeta):

    def __init__(self):
        self.state = 0

    def parse(self, char):
        byte = ord(char)

        if self.state == 0:  # sync char 1
            if byte == 36:  # $
                self.state += 1

        elif self.state == 1:  # sync char 2
            if byte == 77:  # M
                self.state += 1
            else:  # restart and try again
                self.state = 0

        elif self.state == 2:  # direction
            if byte == 62:  # >
                self.message_direction = 1
            else:  # <
                self.message_direction = 0
            self.state += 1

        elif self.state == 3:
            self.message_length_expected = byte
            self.message_checksum = byte
            # setup arraybuffer
            self.message_buffer = b""
            self.state += 1

        elif self.state == 4:
            self.message_id = byte
            self.message_length_received = 0
            self.message_checksum ^= byte
            if self.message_length_expected > 0:
                # process payload
                self.state += 1
            else:
                # no payload
                self.state += 2

        elif self.state == 5:  # payload
            self.message_buffer += char
            self.message_checksum ^= byte
            self.message_length_received += 1
            if self.message_length_received >= self.message_length_expected:
                self.state += 1

        elif self.state == 6:
            if self.message_checksum == byte:
                # message received, process
                self.dispatchMessage()
            else:
                print("code: " + str(self.message_id) + " - crc failed")
            # Reset variables
            self.message_length_received = 0
            self.state = 0

        else:
            print("Unknown state detected: %d" % self.state)

    @staticmethod
    def crc8(data):
        crc = 0x00
        for c in data:
            crc ^= c
        return crc

    def dispatchMessage(self):

        if self.message_id == 105:
            self.handle_RC(*struct.unpack('=hhhhhh', self.message_buffer))

        if self.message_id == 108:
            self.handle_ATTITUDE(*struct.unpack('=hhh', self.message_buffer))

    @abc.abstractmethod
    def handle_RC(self, c1, c2, c3, c4, c5, c6):
        return

    @abc.abstractmethod
    def handle_ATTITUDE(self, angx, angy, heading):
        return

    @staticmethod
    def serialize_RC_Request():
        msg = '$M<' + chr(0) + chr(105) + chr(105)
        return bytes(msg, 'utf-8')

    @staticmethod
    def serialize_ATTITUDE_Request():
        msg = '$M<' + chr(0) + chr(108) + chr(108)
        return bytes(msg, 'utf-8')

    @staticmethod
    def serialize_SET_MOTOR(m1, m2, m3, m4):
        message_buffer = struct.pack('hhhh', m1, m2, m3, m4)
        msg = [len(message_buffer), 214] + list(message_buffer)
        return bytes([ord('$'), ord('M'), ord('<')] + msg + [MspParser.crc8(msg)])