#  MSP Parser subclass and message builders

#  MIT License

import struct

import abc
from msp import Parser


class MspParser(Parser, metaclass=abc.ABCMeta):

    def dispatchMessage(self):

        if self.message_id == 121:
            self.handle_RC_NORMAL(*struct.unpack('=ffffff',
                                  self.message_buffer))

        if self.message_id == 122:
            self.handle_ATTITUDE_RADIANS(*struct.unpack('=fff',
                                         self.message_buffer))

    @abc.abstractmethod
    def handle_RC_NORMAL(self, c1, c2, c3, c4, c5, c6):
        return

    @abc.abstractmethod
    def handle_ATTITUDE_RADIANS(self, roll, pitch, yaw):
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
    def serialize_SET_MOTOR_NORMAL(m1, m2, m3, m4):
        message_buffer = struct.pack('ffff', m1, m2, m3, m4)
        msg = [len(message_buffer), 215] + list(message_buffer)
        return bytes([ord('$'), ord('M'), ord('<')] + msg + [Parser.crc8(msg)])
