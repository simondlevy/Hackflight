#!/usr/bin/python3

'''
   Copyright (c) 2025 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
'''

import json
import argparse
from argparse import ArgumentDefaultsHelpFormatter


# Code-emitter classes ========================================================


class CodeEmitter(object):

    def __init__(self, msgdict, typevals):

        self.msgdict = msgdict
        self.typedict = CodeEmitter._makedict(typevals)
        self.sizedict = CodeEmitter._makedict((1, 2, 4, 4))

    @staticmethod
    def _makedict(items):
        typenames = ('byte', 'short', 'float', 'int')
        return {n: t for n, t in zip(typenames, items)}

    @staticmethod
    def clean(string):
        cleaned_string = string[1: len(string) - 1]
        return cleaned_string

    def _openw(self, fname):

        print('Creating file ' + fname)
        return open(fname, 'w')

    def _paysize(self, argtypes):

        return sum([self.sizedict[atype] for atype in argtypes])

    def _msgsize(self, argtypes):

        return self._paysize(argtypes)

    def _getargnames(self, message):

        return [argname for (argname, _) in self._getargs(message)]

    def _getargtypes(self, message):

        return [argtype for (_, argtype) in self._getargs(message)]

    def _getargs(self, message):

        return [(argname, argtype) for (argname, argtype) in
                zip(message[1], message[2]) if argname.lower() != 'comment']

    def _write_params(self, outfile, argtypes, argnames, prefix='(',
                      ampersand=''):

        outfile.write(prefix)
        for argtype, argname in zip(argtypes, argnames):
            outfile.write(self.typedict[argtype] + ' ' + ampersand + ' ' +
                          argname)
            if argname != argnames[-1]:
                outfile.write(', ')
        outfile.write(')')


# Python emitter ==============================================================


class PythonEmitter(CodeEmitter):

    def __init__(self, msgdict):

        CodeEmitter.__init__(self, msgdict, ('B', 'h', 'f', 'i'))

    def emit(self):

        # Open output file
        self.output = self._openw('msp.py')

        # Emit header
        self.output.write('#  MSP Parser subclass and message builders')
        self.output.write('\n\n#  Copyright (C) 2025 Simon D. Levy')
        self.output.write('\n\n#  AUTO-GENERATED CODE; DO NOT MODIFY')
        self.output.write('\n\n#  Gnu Public License')
        self._write('\n\nimport struct')
        self._write('\n\nimport abc')
        self._write('\n\n\nclass Parser(metaclass=abc.ABCMeta):')

        # Emit error codes
        self._write('\n\n    ERROR_NONE  = 0')
        self._write('\n    ERROR_CRC   = 1')
        self._write('\n    ERROR_STATE = 2')

        # Emit __init__() method
        self._write('\n\n    def __init__(self):')
        self._write('\n        self.state = 0')

        # Emit parse() method
        self._write('\n\n    def parse(self, char):')
        self._write('\n\n        byte = ord(char)')
        self._write('\n\n        error = self.ERROR_NONE\n')
        self._write('\n        if self.state == 0:  # sync char 1')
        self._write('\n            if byte == 36:  # $')
        self._write('\n                self.state += 1\n')
        self._write('\n        elif self.state == 1:  # sync char 2')
        self._write('\n            if byte == 77:  # M')
        self._write('\n                self.state += 1')
        self._write('\n            else:  # restart and try again')
        self._write('\n                self.state = 0\n')
        self._write('\n        elif self.state == 2:  # direction')
        self._write('\n            if byte == 62:  # >')
        self._write('\n                self.message_direction = 1')
        self._write('\n            else:  # <')
        self._write('\n                self.message_direction = 0')
        self._write('\n            self.state += 1\n')
        self._write('\n        elif self.state == 3:')
        self._write('\n            self.message_length_expected = byte')
        self._write('\n            self.message_checksum = byte')
        self._write('\n            # setup arraybuffer')
        self._write('\n            self.message_buffer = b""')
        self._write('\n            self.state += 1\n')
        self._write('\n        elif self.state == 4:')
        self._write('\n            self.message_id = byte')
        self._write('\n            self.message_length_received = 0')
        self._write('\n            self.message_checksum ^= byte')
        self._write('\n            if self.message_length_expected > 0:')
        self._write('\n                # process payload')
        self._write('\n                self.state += 1')
        self._write('\n            else:')
        self._write('\n                # no payload')
        self._write('\n                self.state += 2\n')
        self._write('\n        elif self.state == 5:  # payload')
        self._write('\n            self.message_buffer += char')
        self._write('\n            self.message_checksum ^= byte')
        self._write('\n            self.message_length_received += 1')
        self._write('\n            if self.message_length_received >= self.message_length_expected:')
        self._write('\n                self.state += 1\n')
        self._write('\n        elif self.state == 6:')
        self._write('\n            if self.message_checksum == byte:')
        self._write('\n                # message received, process')
        self._write('\n                self.dispatchMessage()')
        self._write('\n            else:')
        self._write('\n                error = self.ERROR_CRC')
        self._write('\n            # Reset variables')
        self._write('\n            self.message_length_received = 0')
        self._write('\n            self.state = 0\n')
        self._write('\n        else:')
        self._write('\n            error = self.ERROR_STATE')
        self._write('\n\n        return error')

        # Emit crc8() method
        self._write('\n\n    @staticmethod')
        self._write('\n    def crc8(data):')
        self._write('\n        crc = 0x00')
        self._write('\n        for c in data:')
        self._write('\n            crc ^= c')
        self._write('\n        return crc')

        # Emit dispatchMeessage() method
        self._write('\n\n    def dispatchMessage(self):')
        for msgtype in self.msgdict.keys():
            msgstuff = self.msgdict[msgtype]
            msgid = msgstuff[0]
            if msgid < 200:
                self._write('\n\n        if self.message_id == %d:\n'
                            % msgstuff[0])
                self._write('            self.handle_%s(*struct.unpack(\'=' %
                            msgtype)
                for argtype in self._getargtypes(msgstuff):
                    self._write('%s' % self.typedict[argtype])
                self._write("\'" + ', self.message_buffer))')
        self._write('\n\n        return')

        # Emit handler methods for parser
        for msgtype in self.msgdict.keys():

            msgstuff = self.msgdict[msgtype]
            msgid = msgstuff[0]
            if msgid < 200:
                self._write('\n\n    @abc.abstractmethod')
                self._write('\n    def handle_%s(self' % msgtype)
                for argname in self._getargnames(msgstuff):
                    self._write(', ' + argname)
                self._write('):\n')
                self._write('        return')

        # Emit serializer functions for module
        for msgtype in self.msgdict.keys():

            msgstuff = self.msgdict[msgtype]
            msgid = msgstuff[0]

            self._write('\n\n    @staticmethod')

            if msgid < 200:

                self._write('\n    def serialize_' + msgtype + '_Request():\n')
                self._write(('        msg = \'$M<\' + chr(0) + '
                            'chr(%s) + chr(%s)\n') % (msgid, msgid))
                self._write('        return bytes(msg, \'utf-8\')')

            else:

                self._write('\n    def serialize_' + msgtype +
                            '(' + ', '.join(self._getargnames(msgstuff)) +
                            '):\n')
                self._write('        message_buffer = struct.pack(\'')
                for argtype in self._getargtypes(msgstuff):
                    self._write(self.typedict[argtype])
                self._write('\'')
                for argname in self._getargnames(msgstuff):
                    self._write(', ' + argname)
                self._write(')\n')

                self._write(('        msg = [len(message_buffer), %s] + ' +
                            'list(message_buffer)\n') % msgid)
                self._write('        return bytes([ord(\'$\'), ord(\'M\'), ' +
                            'ord(\'<\')] + msg + [Parser.crc8(msg)])')
        self._write('\n')

    def _write(self, s):

        self.output.write(s)

# Java emitter ================================================================


class JavaEmitter(CodeEmitter):

    def __init__(self, msgdict):

        CodeEmitter.__init__(self, msgdict, ('byte', 'short', 'float', 'int'))

        self.bbdict = CodeEmitter._makedict(('', 'Short', 'Float', 'Int'))

    def emit(self):

        self.output = self._openw('MspParser.java')

        # Write header
        self.output.write('/*\n')
        self.output.write('   Message dispatcher\n\n')
        self.output.write('   Gnu Public License\n\n')
        self.output.write('*/\n\n')
        self._write('import edu.wlu.cs.mssppg.Parser;\n\n')
        self._write('public class MspParser extends Parser {\n\n')
        self._write('    protected void dispatchMessage(void) {\n\n')
        self._write('        switch (_command) {\n\n')

        # Write handler cases for incoming messages
        for msgtype in self.msgdict.keys():

            msgstuff = self.msgdict[msgtype]
            msgid = msgstuff[0]

            if msgid < 200:

                self._write('            case (byte)%d:\n' % msgid)
                self._write('                this.handle_%s(\n' % msgtype)

                argnames = self._getargnames(msgstuff)
                argtypes = self._getargtypes(msgstuff)

                nargs = len(argnames)

                offset = 0
                for k in range(nargs):
                    argtype = argtypes[k]
                    self._write('                        bb.get%s(%d)' %
                                (self.bbdict[argtype], offset))
                    offset += self.sizedict[argtype]
                    if k < nargs-1:
                        self._write(',\n')
                self._write(');\n')

                self._write('                break;\n\n')

        self._write('        }\n    }\n\n')

        for msgtype in self.msgdict.keys():

            msgstuff = self.msgdict[msgtype]
            msgid = msgstuff[0]

            argnames = self._getargnames(msgstuff)
            argtypes = self._getargtypes(msgstuff)

            # For messages from FC
            if msgid < 200:

                # Write serializer for requests
                self._write(('    public static byte [] ' +
                            '{serialize_%s_Request() \n\n') % msgtype)
                self._write('        byte [] message = new byte[6];\n\n')
                self._write('        message[0] = 36;\n')
                self._write('        message[1] = 77;\n')
                self._write('        message[2] = 60;\n')
                self._write('        message[3] = 0;\n')
                self._write('        message[4] = (byte)%d;\n' % msgid)
                self._write('        message[5] = (byte)%d;\n\n' % msgid)
                self._write('        return message;\n')
                self._write('    }\n\n')

                # Write handler for replies from flight controller
                self._write('    protected void handle_%s' % msgtype)
                self._write_params(self.output, argtypes, argnames)
                self._write(' { \n        // XXX\n    }\n\n')

        self._write('}\n')

    def _write(self, s):

        self.output.write(s)

# C++ header emitter ==========================================================


class CppHeaderEmitter:

    def __init__(self, msgdict):

        self.msgdict = msgdict

    def emit(self):

        fname = 'messages.h'
        print('Creating file ' + fname)

        with open(fname, 'w') as output:

            output.write('/*\n')
            output.write('MSP message IDs\n')
            output.write('Copyright (C) 2025 Simon D. Levy\n')
            output.write('AUTO-GENERATED CODE; DO NOT MODIFY\n')
            output.write('Gnu Public License\n')
            output.write('*/\n\n')
            output.write('#pragma once\n\n')

            for key in self.msgdict:
                output.write('static const uint8_t MSP_%-20s = %d;\n' %
                             (key, self.msgdict[key][0]))


# main ========================================================================


def main():

    # parse file name from command line
    argparser = argparse.ArgumentParser(
            formatter_class=ArgumentDefaultsHelpFormatter)
    argparser.add_argument('--infile', type=str, required=False,
                           default='messages.json',
                           help='Input file')
    args = argparser.parse_args()

    data = json.load(open(args.infile, 'r'))

    # takes the types of messages from the json file
    unicode_message_types = data.keys()

    # make a list of messages from the JSON file
    message_type_list = list()
    for key in unicode_message_types:
        message_type = json.dumps(key)
        clean_type = CodeEmitter.clean(message_type)
        message_type_list.append(clean_type)

    # make dictionary of names, types for each message's components
    argument_lists = list()
    argument_types = list()
    msgdict = {}
    for msgtype in message_type_list:
        argnames = list()
        argtypes = list()
        msgid = None
        for arg in data[msgtype]:
            argname = CodeEmitter.clean(CodeEmitter.clean(
                json.dumps(list(arg.keys()))))
            argtype = arg[list(arg.keys())[0]]
            if argname == 'ID':
                msgid = int(argtype)
            else:
                argtypes.append(argtype)
                argnames.append(argname)
            argument_lists.append(argnames)
        if msgid is None:
            print('Missing ID for message ' + msgtype)
            exit(1)
        argument_types.append(argtypes)
        msgdict[msgtype] = (msgid, argnames, argtypes)

    # Emit Python
    PythonEmitter(msgdict).emit()

    # Emit Java
    JavaEmitter(msgdict).emit()

    # Emit C++ header
    CppHeaderEmitter(msgdict).emit()


if __name__ == '__main__':
    main()
