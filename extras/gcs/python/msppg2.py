'''
msppg2.py Temporary hand-coded MSP parser

Copyright (C) Simon D. Levy 2018

This program is part of Hackflight

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http:#www.gnu.org/licenses/>.
'''

import struct
import sys

def _CRC8(data):

    crc = 0x00
   
    for c in data:

        crc ^= ord(c) if sys.version[0] == '2' else c

    return crc

class Parser(object):

    def __init__(self):

        self.state = 0

    def parse(self, char):
        '''
        Parses one character, triggering pre-set handlers upon a successful parse.
        '''

        byte = ord(char)

        if self.state ==  0: # sync char 1
            if byte == 36: # $
                self.state += 1

        elif self.state ==  1: # sync char 2
            if byte == 77: # M
                self.state += 1
            else: # restart and try again
                self.state = 0

        elif self.state ==  2: # direction
            if byte == 62:  # >
                self.message_direction = 1
            else: # <
                self.message_direction = 0
            self.state += 1
            
        elif self.state ==  3:
            self.message_length_expected = byte
            self.message_checksum = byte
            # setup arraybuffer
            self.message_buffer = b''
            self.state += 1

        elif self.state ==  4:
            self.message_id = byte
            self.message_length_received  = 0
            self.message_checksum ^= byte
            if self.message_length_expected > 0:
                # process payload
                self.state += 1
            else:
                # no payload
                self.state += 2

        elif self.state ==  5: # payload
            self.message_buffer += char
            self.message_checksum ^= byte
            self.message_length_received += 1
            if self.message_length_received >= self.message_length_expected:
                self.state += 1

        elif self.state ==  6:
            if self.message_checksum == byte:
                # message received, process

                if self.message_id == 112:

                    if self.message_direction == 0:

                        if hasattr(self, 'STATE_Request_Handler'):

                            self.STATE_Request_Handler()

                    else:

                        if hasattr(self, 'STATE_Handler'):

                            self.STATE_Handler(*struct.unpack('=fffffff', self.message_buffer))

                if self.message_id == 121:

                    if self.message_direction == 0:

                        if hasattr(self, 'RC_NORMAL_Request_Handler'):

                            self.RC_NORMAL_Request_Handler()

                    else:

                        if hasattr(self, 'RC_NORMAL_Handler'):

                            self.RC_NORMAL_Handler(*struct.unpack('=ffffff', self.message_buffer))

                if self.message_id == 122:

                    if self.message_direction == 0:

                        self.handle_ATTITUDE_RADIANS_Request()

                        '''
                        if hasattr(self, 'ATTITUDE_RADIANS_Request_Handler'):

                            self.ATTITUDE_RADIANS_Request_Handler()
                        '''

                    else:

                        self.handle_ATTITUDE_RADIANS(*struct.unpack('=fff', self.message_buffer))

                        '''
                        if hasattr(self, 'ATTITUDE_RADIANS_Handler'):

                            self.ATTITUDE_RADIANS_Handler(*struct.unpack('=fff', self.message_buffer))
                        '''

            else:
                print('code: ' + str(self.message_id) + ' - crc failed')
            # Reset variables
            self.message_length_received = 0
            self.state = 0

        else:
            print('Unknown state detected: %d' % self.state)



    def set_STATE_Handler(self, handler):

        '''
        Sets the handler method for when a STATE message is successfully parsed.
        You should declare this message with the following parameter(s):
            altitude,variometer,positionX,positionY,heading,velocityForward,velocityRightward
        '''
        self.STATE_Handler = handler

    def set_RC_NORMAL_Handler(self, handler):

        '''
        Sets the handler method for when a RC_NORMAL message is successfully parsed.
        You should declare this message with the following parameter(s):
            c1,c2,c3,c4,c5,c6
        '''
        self.RC_NORMAL_Handler = handler

    def handle_ATTITUDE_RADIANS(self, x, y, z):

        return

    def set_ATTITUDE_RADIANS_Handler(self, handler):

        '''
        Sets the handler method for when a ATTITUDE_RADIANS message is successfully parsed.
        You should declare this message with the following parameter(s):
            roll,pitch,yaw
        '''
        self.ATTITUDE_RADIANS_Handler = handler

def serialize_STATE(altitude, variometer, positionX, positionY, heading, velocityForward, velocityRightward):
    '''
    Serializes the contents of a message of type STATE.
    '''
    message_buffer = struct.pack('fffffff', altitude, variometer, positionX, positionY, heading, velocityForward, velocityRightward)

    if sys.version[0] == '2':
        msg = chr(len(message_buffer)) + chr(112) + str(message_buffer)
        return '$M>' + msg + chr(_CRC8(msg))

    else:
        msg = [len(message_buffer), 112] + list(message_buffer)
        return bytes([ord('$'), ord('M'), ord('<')] + msg + [_CRC8(msg)])

def serialize_STATE_Request():

    '''
    Serializes a request for STATE data.
    '''
    msg = '$M<' + chr(0) + chr(112) + chr(112)
    return bytes(msg) if sys.version[0] == '2' else bytes(msg, 'utf-8')

def serialize_RC_NORMAL(c1, c2, c3, c4, c5, c6):
    '''
    Serializes the contents of a message of type RC_NORMAL.
    '''
    message_buffer = struct.pack('ffffff', c1, c2, c3, c4, c5, c6)

    if sys.version[0] == '2':
        msg = chr(len(message_buffer)) + chr(121) + str(message_buffer)
        return '$M>' + msg + chr(_CRC8(msg))

    else:
        msg = [len(message_buffer), 121] + list(message_buffer)
        return bytes([ord('$'), ord('M'), ord('<')] + msg + [_CRC8(msg)])

def serialize_RC_NORMAL_Request():

    '''
    Serializes a request for RC_NORMAL data.
    '''
    msg = '$M<' + chr(0) + chr(121) + chr(121)
    return bytes(msg) if sys.version[0] == '2' else bytes(msg, 'utf-8')

def serialize_ATTITUDE_RADIANS(roll, pitch, yaw):
    '''
    Serializes the contents of a message of type ATTITUDE_RADIANS.
    '''
    message_buffer = struct.pack('fff', roll, pitch, yaw)

    if sys.version[0] == '2':
        msg = chr(len(message_buffer)) + chr(122) + str(message_buffer)
        return '$M>' + msg + chr(_CRC8(msg))

    else:
        msg = [len(message_buffer), 122] + list(message_buffer)
        return bytes([ord('$'), ord('M'), ord('<')] + msg + [_CRC8(msg)])

def serialize_ATTITUDE_RADIANS_Request():

    '''
    Serializes a request for ATTITUDE_RADIANS data.
    '''
    msg = '$M<' + chr(0) + chr(122) + chr(122)
    return bytes(msg) if sys.version[0] == '2' else bytes(msg, 'utf-8')

def serialize_SET_MOTOR_NORMAL(m1, m2, m3, m4):
    '''
    Serializes the contents of a message of type SET_MOTOR_NORMAL.
    '''
    message_buffer = struct.pack('ffff', m1, m2, m3, m4)

    if sys.version[0] == '2':
        msg = chr(len(message_buffer)) + chr(215) + str(message_buffer)
        return '$M<' + msg + chr(_CRC8(msg))

    else:
        msg = [len(message_buffer), 215] + list(message_buffer)
        return bytes([ord('$'), ord('M'), ord('<')] + msg + [_CRC8(msg)])

def serialize_SET_ARMED(flag):
    '''
    Serializes the contents of a message of type SET_ARMED.
    '''
    message_buffer = struct.pack('B', flag)

    if sys.version[0] == '2':
        msg = chr(len(message_buffer)) + chr(216) + str(message_buffer)
        return '$M<' + msg + chr(_CRC8(msg))

    else:
        msg = [len(message_buffer), 216] + list(message_buffer)
        return bytes([ord('$'), ord('M'), ord('<')] + msg + [_CRC8(msg)])

def serialize_SET_RC_NORMAL(c1, c2, c3, c4, c5, c6):
    '''
    Serializes the contents of a message of type SET_RC_NORMAL.
    '''
    message_buffer = struct.pack('ffffff', c1, c2, c3, c4, c5, c6)

    if sys.version[0] == '2':
        msg = chr(len(message_buffer)) + chr(222) + str(message_buffer)
        return '$M<' + msg + chr(_CRC8(msg))

    else:
        msg = [len(message_buffer), 222] + list(message_buffer)
        return bytes([ord('$'), ord('M'), ord('<')] + msg + [_CRC8(msg)])

def serialize_SET_RANGE_AND_FLOW(range, flowx, flowy):
    '''
    Serializes the contents of a message of type SET_RANGE_AND_FLOW.
    '''
    message_buffer = struct.pack('hhh', range, flowx, flowy)

    if sys.version[0] == '2':
        msg = chr(len(message_buffer)) + chr(226) + str(message_buffer)
        return '$M<' + msg + chr(_CRC8(msg))

    else:
        msg = [len(message_buffer), 226] + list(message_buffer)
        return bytes([ord('$'), ord('M'), ord('<')] + msg + [_CRC8(msg)])

