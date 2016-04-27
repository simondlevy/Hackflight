from struct import unpack

class NAV_POSLLH(object):

    def __init__(self, iTOW, lon, lat, height, hMSL, hAcc, vAcc):
    
        self.iTOW   = iTOW     
        self.lon    = lon     
        self.lat    = lat     
        self.height = height     
        self.hMSL   = hMSL     
        self.hAcc   = hAcc
        self.vAcc   = vAcc

    def __str__(self):

        return 'NAV-POSLLH: iTOW=%d ms|lon=%f deg|lat=%f deg|height=%d mm|hMSL=%d mm|hAcc=%d mm|vAcc=%d mm' % \
                 (self.iTOW, self.lon/1e7, self.lat/1e7, self.height, self.hMSL, self.hAcc, self.vAcc)

class NAV_DOP(object):

    def __init__(self, iTOW, gDOP, pDOP, tDOP, vDOP, hDOP, nDOP, eDOP):

        self.iTOW = iTOW
        self.gDOP = gDOP
        self.pDOP = pDOP
        self.tDOP = tDOP
        self.vDOP = vDOP
        self.hDOP = hDOP
        self.nDOP = nDOP
        self.eDOP = eDOP

    def __str__(self):

        return 'NAV-DOP: iTOW=%d ms|gDOP=%3.3f|pDOP=%3.3f|tDOP=%3.3f|vDOP=%3.3f|hDOP=%3.3f|nDOP=%3.3f|eDOP=%3.3f' % \
                 (self.iTOW, self.gDOP/1e2, self.pDOP/1e2, self.tDOP/1e2, self.vDOP/1e2, self.hDOP/1e2, self.nDOP/1e2, self.eDOP/1e2)

class NAV_VELNED(object):

    def __init__(self, iTOW, velN, velE, velD, speed, gSpeed, heading, sAcc, cAcc):

        self.iTOW = iTOW
        self.velN = velN
        self.velE = velE
        self.velD = velD
        self.speed = speed
        self.gSpeed = gSpeed
        self.heading = heading
        self.sAcc = sAcc
        self.cAcc = cAcc

    def __str__(self):

        return 'NAV-VELNED: iTOW=%d ms|velN=%d cm/s|velE=%d cm/s|velD=%d cm/s|speed=%d cm/s|gSpeed=%d cm/s|heading=%f deg|sAcc=%d cm/s|cAcc=%f deg' % \
                 (self.iTOW, self.velN, self.velE, self.velD, self.speed, self.gSpeed, self.heading/1e5, self.sAcc, self.cAcc/1e5)

class UBXParser(object):

    def __init__(self):

        self.STATE_GOT_SYNC1   = 0
        self.STATE_GOT_SYNC2   = 1
        self.STATE_GOT_CLASS   = 2
        self.STATE_GOT_ID      = 3
        self.STATE_GOT_LENGTH1 = 4
        self.STATE_GOT_LENGTH2 = 5
        self.STATE_GOT_PAYLOAD = 6
        self.STATE_GOT_CHKA    = 7

        self.state    = -1
        self.msgclass = -1
        self.msgid    = -1
        self.msglen   = -1
        self.chka     = -1
        self.chkb     = -1
        self.payload  = -1
        self.count    = -1


    def parse(self, c):

        b = ord(c)

        if b == 0xB5:

            self.state = self.STATE_GOT_SYNC1

        elif b == 0x62 and self.state == self.STATE_GOT_SYNC1:

            self.state = self.STATE_GOT_SYNC2
            self.chka = 0
            self.chkb = 0

        elif self.state == self.STATE_GOT_SYNC2:

            self.state = self.STATE_GOT_CLASS
            self.msgclass = b
            self._addchk(b)

        elif self.state == self.STATE_GOT_CLASS:

            self.state = self.STATE_GOT_ID
            self.msgid = b
            self._addchk(b)

        elif self.state == self.STATE_GOT_ID:

            self.state = self.STATE_GOT_LENGTH1
            self.msglen = b
            self._addchk(b)

        elif self.state == self.STATE_GOT_LENGTH1:

            self.state = self.STATE_GOT_LENGTH2
            self.msglen += (b << 8)
            self.payload = ''
            self.count = 0
            self._addchk(b)
 
        elif self.state == self.STATE_GOT_LENGTH2:

            self.count += 1
            self._addchk(b)
            self.payload += c
            
            if self.count == self.msglen:

                self.state = self.STATE_GOT_PAYLOAD
 
        elif self.state == self.STATE_GOT_PAYLOAD:

            self._checkchk(b, self.chka)
            self.state = self.STATE_GOT_CHKA

        elif self.state == self.STATE_GOT_CHKA:

            self._checkchk(b, self.chkb)

            return self._factory()

        return None

    def _addchk(self, b):

        self.chka = (self.chka + b)         & 0xFF
        self.chkb = (self.chkb + self.chka) & 0xFF

    def _checkchk(self, b, chk):

        if b != chk:
           raise Exception('Checksum mismatch')

    def _factory(self):

        if   self.msgid == 0x02:

            iTOW, lon, lat, height, hMSL, hAcc, vAcc = unpack('IiiiiII', self.payload)

            return NAV_POSLLH(iTOW, lon, lat, height, hMSL, hAcc, vAcc)

        elif self.msgid == 0x04:

            iTOW, gDOP, pDOP, tDOP, vDOP, hDOP, nDOP, eDOP = unpack('Ihhhhhhh', self.payload)

            return NAV_DOP(iTOW, gDOP, pDOP, tDOP, vDOP, hDOP, nDOP, eDOP)

        elif self.msgid == 0x12:

            iTOW, velN, velE, velD, speed, gSpeed, heading, sAcc, cAcc = unpack('IiiiIIiII', self.payload)

            return NAV_VELNED(iTOW, velN, velE, velD, speed, gSpeed, heading, sAcc, cAcc)

        else:

            return '0x%02X' % self.msgid
