#!/usr/bin/python3
'''
Multiwii Serial Protocol Parser Generator

Copyright (C) Rob Jones, Alec Singer, Chris Lavin, Blake Liebling, Simon D. Levy 2021

MIT License
'''

import os
import json
import argparse


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


# C++ emitter =================================================================


class Cpp_Emitter(CodeEmitter):

    def __init__(self, msgdict):

        CodeEmitter.__init__(self, msgdict,
                             ('uint8_t', 'int16_t', 'float', 'int32_t'))

    def emit(self):

        # Open output file
        output = self._openw('serialtask.hpp')

        # Write header
        output.write('/*\n')
        output.write('   Timer task for serial comms\n\n')
        output.write('   MIT License\n')
        output.write(' */\n\n')
        output.write('#pragma once\n\n')
        output.write('#include <RFT_board.hpp>\n')
        output.write('#include <RFT_debugger.hpp>\n')
        output.write('#include <RFT_actuator.hpp>\n')
        output.write('#include <RFT_parser.hpp>\n')
        output.write('#include <RFT_serialtask.hpp>\n\n')

        # Add namespace
        output.write('namespace /* XXX */ {\n\n')

        # Add classname
        output.write('\n    class SerialTask : public rft::SerialTask {')

        # Add friend class declaration
        output.write('\n\n        friend class /* XXX */;')

        # Add stubbed declarations for handler methods

        output.write('\n\n        private:\n')
        output.write('\n            uint8_t _payload[128] = {};\n')

        for msgtype in self.msgdict.keys():

            msgstuff = self.msgdict[msgtype]
            msgid = msgstuff[0]

            argnames = self._getargnames(msgstuff)
            argtypes = self._getargtypes(msgstuff)

            output.write('\n            void handle_%s%s' %
                         (msgtype, '_Request' if msgid < 200 else ''))
            self._write_params(output, argtypes, argnames,
                               ampersand=('&' if msgid < 200 else ''))
            output.write('\n            {')
            output.write('\n                // XXX')
            output.write('\n            }\n')

        output.write('\n        protected:\n\n')

        # Add collectPayload() method
        output.write('            virtual void collectPayload(uint8_t index, uint8_t value) override\n')
        output.write('            {\n')
        output.write('                _payload[index] = value;\n')
        output.write('            }\n\n')

        # Add dispatchMessage() method

        output.write('            virtual void dispatchMessage(uint8_t command) override\n')
        output.write('            {\n')
        output.write('                switch (command) {\n\n')

        for msgtype in self.msgdict.keys():

            msgstuff = self.msgdict[msgtype]
            msgid = msgstuff[0]

            argnames = self._getargnames(msgstuff)
            argtypes = self._getargtypes(msgstuff)

            output.write('                    case %s:' %
                         self.msgdict[msgtype][0])
            output.write('\n                        {')
            nargs = len(argnames)
            offset = 0
            for k in range(nargs):
                argname = argnames[k]
                argtype = argtypes[k]
                decl = self.typedict[argtype]
                output.write('\n                            ' + 
                             decl + ' ' + argname + ' = 0;')
                if msgid >= 200:
                    fmt = 'memcpy(&%s,  &_payload[%d], sizeof(%s));\n'
                    output.write('\n                            ')
                    output.write(fmt % (argname, offset, decl))
                offset += self.sizedict[argtype]
            output.write('\n                            handle_%s%s(' %
                         (msgtype, '_Request' if msgid < 200 else ''))
            for k in range(nargs):
                output.write(argnames[k])
                if k < nargs-1:
                    output.write(', ')
            output.write(');\n')
            if msgid < 200:
                # XXX enforce uniform type for now
                argtype = argtypes[0].capitalize()
                output.write('                            ')
                output.write('prepareToSend%ss(command, %d);\n' % (argtype, nargs))
                for argname in argnames:
                    output.write('                            send%s(%s);\n' %
                                 (argtype, argname))
                output.write('                            ')
                output.write('completeSend();\n')
            output.write('                        } break;\n\n')

        output.write('                } // switch (_command)\n\n')
        output.write('            } // dispatchMessage \n\n')
        output.write('        }; // class SerialTask\n\n')
        output.write('} // namespace XXX\n')


# Python emitter ==============================================================


class Python_Emitter(CodeEmitter):

    def __init__(self, msgdict):

        CodeEmitter.__init__(self, msgdict, ('B', 'h', 'f', 'i'))

    def emit(self):

        # Open output file
        self.output = self._openw('mspparser.py')

        # Emit header
        self.output.write('#  MSP Parser subclass and message builders')
        self.output.write('\n\n#  Copyright (C) 2021 Simon D. Levy')
        self.output.write('\n\n#  AUTO-GENERATED CODE; DO NOT MODIFY')
        self.output.write('\n\n#  MIT License')
        self._write('\n\nimport struct')
        self._write('\n\nimport abc')
        self._write('\n\n\nclass MspParser(metaclass=abc.ABCMeta):')

        # Emit __init__() method
        self._write('\n\n    def __init__(self):')
        self._write('\n        self.state = 0')

        # Emit parse() method
        self._write('\n\n    def parse(self, char):')
        self._write('\n        byte = ord(char)\n')
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
        self._write('\n                print("code: " + str(self.message_id) + " - crc failed")')
        self._write('\n            # Reset variables')
        self._write('\n            self.message_length_received = 0')
        self._write('\n            self.state = 0\n')
        self._write('\n        else:')
        self._write('\n            print("Unknown state detected: %d" % self.state)')

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
                            'ord(\'<\')] + msg + [MspParser.crc8(msg)])')
        self._write('\n')

    def _write(self, s):

        self.output.write(s)

# Java emitter ================================================================


class Java_Emitter(CodeEmitter):

    def __init__(self, msgdict):

        CodeEmitter.__init__(self, msgdict, ('byte', 'short', 'float', 'int'))

        self.bbdict = CodeEmitter._makedict(('', 'Short', 'Float', 'Int'))

    def emit(self):

        self.output = self._openw('MspParser.java')

        # Write header
        self.output.write('/*\n')
        self.output.write('   Message dispatcher\n\n')
        self.output.write('   MIT License\n\n')
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

# main ========================================================================


def main():

    # parse file name from command line
    parser = argparse.ArgumentParser()
    parser.add_argument('--infile', type=str, required=False,
                        default='messages.json',
                        help='Random seed for reproducibility')
    args = parser.parse_args()

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
    Python_Emitter(msgdict).emit()


if __name__ == '__main__':
    main()
