'''

FrSky telemetry parser

  Approximately matches www.frsky-rc.com/download/down.php?id=126, but we appear to
  be getting four frame types instead of three.  So we identify the message by the
  hex code of its first byte.

'''

from struct import unpack

class _Message(object):

    def __init__(self, code):

        self.code = code

    def _chkbytestuff(self, data):

        return (data[2]^0x60, data[3]) if data[1] == 0x5D else data[1:3]

    def _signed_short(self, data):

        data = self._chkbytestuff(data)

        return unpack('h', chr(data[0])+chr(data[1]))[0]

    def _unsigned_short(self, data):

        data = self._chkbytestuff(data)

        return unpack('H', chr(data[0])+chr(data[1]))[0]

class Message0x17(_Message):

    def __init__(self, frame):

        _Message.__init__(self, 0x17)

        # XXX maybe should run _chkbytestuff on this
        self.minutes = frame[0][2]
        self.seconds = frame[1][1]

    def __str__(self):

        return 'Message 0x17: Time = %03d:%02d' % (self.minutes, self.seconds)

class Message0x10(_Message):

    def __init__(self, frame):

        _Message.__init__(self, 0x10)

        self.altitude = self._unsigned_short(frame[1])

        heading = self._chkbytestuff(frame[2])
        self.heading = 255 * heading[1] + heading[0] 

    def __str__(self):

        return 'Message 0x10: Altitude: %d m | Heading: %d deg' % (self.altitude, self.heading)

class Message0x24(_Message):

    def __init__(self, frame):

        _Message.__init__(self, 0x24)

        self.acc_x = self._signed_short(frame[0])
        self.acc_y = self._signed_short(frame[1])
        self.acc_z = self._signed_short(frame[2])

    def __str__(self):

        return 'Message 0x24: Accelerometer: X=%+04d Y=%+04d Z=%+04d' % (self.acc_x, self.acc_y, self.acc_z)

class Message0x02(_Message):

    def __init__(self, frame):

        _Message.__init__(self, 0x02)

    def __str__(self):

        return 'Message 0x02:'

class FrSkyTelemetryParser(object):

    def __init__(self):

        # Implements simple state machine
        self.prev = None
        self.begun = False

        self.frame = ''

    def parse(self, c):

        retval = None

        b = ord(c)

        if b == 0x5E and self.prev == 0x5E:

            if self.begun:

                frame = [[ord(x) for x in elem] for elem in self.frame[:-1].split('\x5E')]

                self.frame = ''

                dataD1 = frame[0][0]

                if dataD1 == 0x02:

                    retval = Message0x02(frame)

                elif dataD1 == 0x10:

                    retval = Message0x10(frame)

                elif dataD1 == 0x17:

                    retval = Message0x17(frame)

                elif dataD1 == 0x24:

                    retval = Message0x24(frame)

            self.begun = True

        elif self.begun:

            self.frame += c

        self.prev = b

        return retval
