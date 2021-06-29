#  MSP Parser subclass and message builders

#  MIT License

import struct

import abc

class MspParser(metaclass=abc.ABCMeta):

    def __init__(self):

        self.state = 0

    def parse(self, char):
        '''
        Parses one character, triggering pre-set handlers upon a successful
        parse.
        '''

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
            self.message_buffer = b''
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
                print('code: ' + str(self.message_id) + ' - crc failed')
            # Reset variables
            self.message_length_received = 0
            self.state = 0

        else:
            print('Unknown state detected: %d' % self.state)

    @staticmethod
    def crc8(data):

        crc = 0x00

        for c in data:

            crc ^= c

        return crc

    def dispatchMessage(self):

        if self.message_id == 121:
            self.handle_RC_NORMAL(*struct.unpack('=ffffff',
                                                 self.message_buffer))

        if self.message_id == 122:
            self.handle_ATTITUDE_RADIANS(*struct.unpack('=fff',
                                                        self.message_buffer))

        if self.message_id == 123:
            self.handle_ACTUATOR_TYPE(*struct.unpack('=B',
                                                     self.message_buffer))

    @abc.abstractmethod
    def handle_RC_NORMAL(self, c1, c2, c3, c4, c5, c6):
        return

    @abc.abstractmethod
    def handle_ATTITUDE_RADIANS(self, roll, pitch, yaw):
        return

    @abc.abstractmethod
    def handle_ACTUATOR_TYPE(self, mtype):
        return

    @staticmethod
    def serialize_RC_NORMAL_Request():
        msg = '$M<' + chr(0) + chr(121) + chr(121)
        return bytes(msg, 'utf-8')

    @staticmethod
    def serialize_ATTITUDE_RADIANS_Request():
        msg = '$M<' + chr(0) + chr(122) + chr(122)
        return bytes(msg, 'utf-8')

    @staticmethod
    def serialize_ACTUATOR_TYPE_Request():
        msg = '$M<' + chr(0) + chr(123) + chr(123)
        return bytes(msg, 'utf-8')

    @staticmethod
    def serialize_SET_MOTOR_NORMAL(m1, m2, m3, m4):
        message_buffer = struct.pack('ffff', m1, m2, m3, m4)
        msg = [len(message_buffer), 215] + list(message_buffer)
        return bytes([ord('$'), ord('M'), ord('<')] + msg + [MspParser.crc8(msg)])
