#!/usr/bin/python3

'''
msppg.py Multiwii Serial Protocol Parser Generator

Copyright (C) Rob Jones, Alec Singer, Chris Lavin, Blake Liebling, Simon D. Levy 2015

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

from sys import exit, argv
import os
import json
from pkg_resources import resource_string

# Helper functions ===========================================================================

def clean(string):
    cleaned_string = string[1: len(string) - 1]
    return cleaned_string

def mkdir_if_missing(dirname):
    if not os.path.exists(dirname):
        os.mkdir(dirname)

def error(errmsg):
    print(errmsg)
    exit(1)

def _openw(fname):

    print('Creating file ' + fname)
    return open(fname, 'w')

# Code-emitter classes=======================================================================

class CodeEmitter(object):

    def __init__(self):

        self.indent = '    '
        self.type2size = {'byte': 1, 'short' : 2, 'float' : 4, 'int' : 4}

    def _paysize(self, argtypes):

        return sum([self.type2size[atype] for atype in argtypes])
    
    def _msgsize(self, argtypes):

        return self._paysize(argtypes)

    def _getargnames(self, message):

        return [argname for (argname,_) in self._getargs(message)]

    def _getargtypes(self, message):

        return [argtype for (_,argtype) in self._getargs(message)]

    def _getargs(self, message):

        return [(argname,argtype) for (argname,argtype) in 
                zip(message[1], message[2]) if argname.lower()!='comment']

    def _write_params(self, outfile, argtypes, argnames, prefix = '(', ampersand=''):

        outfile.write(prefix)
        for argtype,argname in zip(argtypes, argnames):
            outfile.write(self.type2decl[argtype] + ' ' +  ampersand + ' ' + argname)
            if argname != argnames[-1]:
                outfile.write(', ')
        outfile.write(')')

    def _getsrc(self, filename):

        return resource_string('resources', filename).decode('utf-8')
 
    def _copyfile(self, src, dst, folder='output'):

        outfile = _openw('%s/%s' % (folder, dst))
        outfile.write(self._getsrc(src))
        outfile.close()

class LocalCodeEmitter(CodeEmitter):

    def __init__(self, folder, ext):

        CodeEmitter.__init__(self)

        mkdir_if_missing('output/%s' % folder)

class CompileableCodeEmitter(LocalCodeEmitter):

    def __init__(self, folder, ext):

        LocalCodeEmitter.__init__(self, folder, ext)

        self._copyfile('%s.makefile' % folder, '%s/Makefile' % folder)

# Python emitter ============================================================================

class Python_Emitter(LocalCodeEmitter):


    def __init__(self, msgdict):

        LocalCodeEmitter.__init__(self, 'python', 'py')

        for filename in os.listdir('./resources'):
            if filename.endswith('.py') and not filename in ['__init__.py', 'setup.py']:
                LocalCodeEmitter._copyfile(self, '%s' % filename, 'python/' + ('%s' % filename))

        mkdir_if_missing('output/python/msppg')

        self._copyfile('setup.py', 'python/setup.py')

        self.output = _openw('output/python/msppg/__init__.py')

        self.type2pack = {'byte' : 'B', 'short' : 'h', 'float' : 'f', 'int' : 'i'}

        self._write(self._getsrc('top-py') + '\n')

        for msgtype in msgdict.keys():
            msgstuff = msgdict[msgtype]
            msgid = msgstuff[0]
            if msgid < 200:
                self._write(4*self.indent + ('if self.message_id == %d:\n\n' % msgstuff[0]))
                self._write(5*self.indent + ('if self.message_direction == 0:\n\n'))
                self._write(6*self.indent + 'if hasattr(self, \'' +  msgtype + '_Request_Handler\'):\n\n')
                self._write(7*self.indent + 'self.%s_Request_Handler()\n\n' % msgtype)
                self._write(5*self.indent + 'else:\n\n')
                self._write(6*self.indent + 'if hasattr(self, \'' +  msgtype + '_Handler\'):\n\n')
                self._write(7*self.indent + 'self.%s_Handler(*struct.unpack(\'=' % msgtype)
                for argtype in self._getargtypes(msgstuff):
                    self._write('%s' % self.type2pack[argtype])
                self._write("\'" + ', self.message_buffer))\n\n')

        self._write(self._getsrc('bottom-py') + '\n')

        # Emit handler methods for parser
        for msgtype in msgdict.keys():

            msgstuff = msgdict[msgtype]
            msgid = msgstuff[0]

            if msgid < 200:

                self._write(self.indent + 'def set_%s_Handler(self, handler):\n\n' % msgtype) 
                self._write(2*self.indent + "'''\n")
                self._write(2*self.indent + 'Sets the handler method for when a %s message is successfully parsed.\n' %
                        msgtype)
                self._write(2*self.indent + 'You should declare this message with the following parameter(s):\n')
                self._write(3*self.indent + ','.join(self._getargnames(msgstuff)))
                self._write('\n' + 2*self.indent + "'''\n")
                self._write(2*self.indent + 'self.%s_Handler = handler\n\n' % msgtype)

                #self._write(self.indent + 'def set_%s_Request_Handler(self, handler):\n\n' % msgtype) 
                #self._write(2*self.indent + 'self.%s_Request_Handler = handler\n\n' % msgtype)

        # Emit serializer functions for module
        for msgtype in msgdict.keys():

            msgstuff = msgdict[msgtype]
            msgid = msgstuff[0]

            self._write('def serialize_' + msgtype + '(' + ', '.join(self._getargnames(msgstuff)) + '):\n')
            self._write(self.indent + "'''\n")
            self._write(self.indent + 'Serializes the contents of a message of type ' + msgtype + '.\n')
            self._write(self.indent + "'''\n")
            self._write(self.indent + 'message_buffer = struct.pack(\'')
            for argtype in self._getargtypes(msgstuff):
                self._write(self.type2pack[argtype])
            self._write('\'')
            for argname in self._getargnames(msgstuff):
                self._write(', ' + argname)
            self._write(')\n\n')
            self._write(self.indent)

            self._write('if sys.version[0] == \'2\':\n')
            self._write(self.indent*2 + 'msg = chr(len(message_buffer)) + chr(%s) + str(message_buffer)\n' % msgid)
            self._write(self.indent*2 + 'return \'$M%c\' + msg + chr(_CRC8(msg))\n\n' % ('>' if msgid < 200 else '<'))
            self._write(self.indent+'else:\n')
            self._write(self.indent*2 + 'msg = [len(message_buffer), %s] + list(message_buffer)\n' % msgid)
            self._write(self.indent*2 + 'return bytes([ord(\'$\'), ord(\'M\'), ord(\'<\')] + msg + [_CRC8(msg)])\n\n')


            if msgid < 200:

                self._write('def serialize_' + msgtype + '_Request():\n\n')
                self._write(self.indent + "'''\n")
                self._write(self.indent + 'Serializes a request for ' + msgtype + ' data.\n')
                self._write(self.indent + "'''\n")
                self._write(self.indent+'msg = \'$M<\' + chr(0) + chr(%s) + chr(%s)\n' % (msgid, msgid))
                self._write(self.indent+'return bytes(msg) if sys.version[0] == \'2\' else bytes(msg, \'utf-8\')\n\n')


    def _write(self, s):

        self.output.write(s)

# Firmware header-only code emitter ===================================================================

class HPP_Emitter(CodeEmitter):

    type2decl    = {'byte': 'uint8_t', 'short' : 'int16_t', 'float' : 'float', 'int' : 'int32_t'}
    type2keyword = {'byte': 'Byte', 'short' : 'Short', 'float' : 'Float', 'int' : 'Int'}

    def __init__(self, msgdict):

        CodeEmitter.__init__(self)

        self.type2decl = HPP_Emitter.type2decl
        self.type2keyword = HPP_Emitter.type2keyword

        # Create C++ header file
        self._copyfile('mspparser.hpp', 'mspparser.hpp', '../../src')

        # Open file for appending
        self.output = open('../../src/mspparser.hpp', 'a')

        # Add dispatchRequestMessage() method

        self.output.write(3*self.indent + 'void dispatchRequestMessage(void)\n')
        self.output.write(3*self.indent + '{\n')
        self.output.write(4*self.indent + 'switch (_command) {\n\n')

        for msgtype in msgdict.keys():

            msgstuff = msgdict[msgtype]
            msgid = msgstuff[0]

            argnames = self._getargnames(msgstuff)
            argtypes = self._getargtypes(msgstuff)

            self.output.write(5*self.indent + ('case %s:\n' % msgdict[msgtype][0]))
            self.output.write(5*self.indent + '{\n')
            nargs = len(argnames)
            offset = 0
            for k in range(nargs):
                argname = argnames[k]
                argtype = argtypes[k]
                decl = self.type2decl[argtype]
                self.output.write(6*self.indent + decl  + ' ' + argname + ' = 0;\n')
                if msgid >= 200:
                    self.output.write(6*self.indent + 'memcpy(&%s,  &_inBuf[%d], sizeof(%s));\n\n' % (argname, offset, decl))
                offset += self.type2size[argtype]
            self.output.write(6*self.indent + 'handle_%s_Request(' %  msgtype)
            for k in range(nargs):
                self.output.write(argnames[k])
                if k < nargs-1:
                    self.output.write(', ')
            self.output.write(');\n')
            if msgid < 200:
                self.output.write(6*self.indent + ('prepareToSend%ss(%d);\n' % (self.type2keyword[argtype], nargs)))
                for argname in argnames:
                    self.output.write(6*self.indent + ('send%s(%s);\n' % (self.type2keyword[argtype], argname)))
                self.output.write(6*self.indent + "serialize8(_checksum);\n")
            self.output.write(6*self.indent + '} break;\n\n')

        self.output.write(4*self.indent + '}\n')
        self.output.write(3*self.indent + '}\n\n')

        # Add dispatchDataMessage() method

        self.output.write(3*self.indent + 'void dispatchDataMessage(void)\n')
        self.output.write(3*self.indent + '{\n')
        self.output.write(4*self.indent + 'switch (_command) {\n\n')

        for msgtype in msgdict.keys():

            msgstuff = msgdict[msgtype]
            msgid = msgstuff[0]

            if msgid < 200:

                argnames = self._getargnames(msgstuff)
                argtypes = self._getargtypes(msgstuff)

                self.output.write(5*self.indent + ('case %s:\n' % msgdict[msgtype][0]))
                self.output.write(5*self.indent + '{\n')
                nargs = len(argnames)
                offset = 0
                for k in range(nargs):
                    argname = argnames[k]
                    argtype = argtypes[k]
                    decl = self.type2decl[argtype]
                    self.output.write(6*self.indent + decl  + ' ' + argname + ' = getArgument(%d);\n' % k)
                    offset += self.type2size[argtype]
                self.output.write(6*self.indent + 'handle_%s_Data(' %  msgtype)
                for k in range(nargs):
                    self.output.write(argnames[k])
                    if k < nargs-1:
                        self.output.write(', ')
                self.output.write(');\n')
                self.output.write(6*self.indent + '} break;\n\n')

        self.output.write(4*self.indent + '}\n')
        self.output.write(3*self.indent + '}\n\n')


        # Add virtual declarations for handler methods

        self.output.write(2*self.indent + 'protected:\n\n')

        for msgtype in msgdict.keys():

            msgstuff = msgdict[msgtype]
            msgid = msgstuff[0]

            argnames = self._getargnames(msgstuff)
            argtypes = self._getargtypes(msgstuff)

            self.output.write(3*self.indent + 'virtual void handle_%s_Request' % msgtype)
            self._write_params(self.output, argtypes, argnames, ampersand = '&' if msgid<200 else '')
            self.output.write('\n' + 3*self.indent + '{\n')
            for argname in argnames:
                self.output.write(4*self.indent + '(void)%s;\n' % argname)
            self.output.write(3*self.indent + '}\n\n')

            self.output.write(3*self.indent + 'virtual void handle_%s_Data' % msgtype)
            self._write_params(self.output, argtypes, argnames, ampersand = '&' if msgid<200 else '')
            self.output.write('\n' + 3*self.indent + '{\n')
            for argname in argnames:
                self.output.write(4*self.indent + '(void)%s;\n' % argname)
            self.output.write(3*self.indent + '}\n\n')

        # Add message-serialization declarations to header

        self.output.write(self.indent*2 + 'public:\n\n')

        for msgtype in msgdict.keys():

            msgstuff = msgdict[msgtype]
            msgid = msgstuff[0]

            argnames = self._getargnames(msgstuff)
            argtypes = self._getargtypes(msgstuff)

            # Incoming messages
            if msgid < 200:

                # Write request method
                self.output.write(3*self.indent + 'static uint8_t serialize_%s_Request(uint8_t bytes[])\n' % msgtype)
                self.output.write(3*self.indent + '{\n')
                self.output.write(4*self.indent + 'bytes[0] = 36;\n')
                self.output.write(4*self.indent + 'bytes[1] = 77;\n')
                self.output.write(4*self.indent + 'bytes[2] = %d;\n' % 60 if msgid < 200 else 62)
                self.output.write(4*self.indent + 'bytes[3] = 0;\n')
                self.output.write(4*self.indent + 'bytes[4] = %d;\n' % msgid)
                self.output.write(4*self.indent + 'bytes[5] = %d;\n\n' % msgid)
                self.output.write(4*self.indent + 'return 6;\n')
                self.output.write(3*self.indent + '}\n\n')

            # Add parser method for serializing message
            self.output.write(3*self.indent + 'static uint8_t serialize_%s' % msgtype)
            self._write_params(self.output, argtypes, argnames, '(uint8_t bytes[], ')
            self.output.write('\n' + 3*self.indent + '{\n')
            msgsize = self._msgsize(argtypes)
            self.output.write(4*self.indent + 'bytes[0] = 36;\n')
            self.output.write(4*self.indent + 'bytes[1] = 77;\n')
            self.output.write(4*self.indent + 'bytes[2] = 62;\n')
            self.output.write(4*self.indent + 'bytes[3] = %d;\n' % msgsize)
            self.output.write(4*self.indent + 'bytes[4] = %d;\n\n' % msgid)
            nargs = len(argnames)
            offset = 5
            for k in range(nargs):
                argname = argnames[k]
                argtype = argtypes[k]
                decl = self.type2decl[argtype]
                self.output.write(4*self.indent + 
                        'memcpy(&bytes[%d], &%s, sizeof(%s));\n' %  (offset, argname, decl))
                offset += self.type2size[argtype]
            self.output.write('\n')
            self.output.write(4*self.indent + 
                    'bytes[%d] = CRC8(&bytes[3], %d);\n\n' % (msgsize+5, msgsize+2))
            self.output.write(4*self.indent + 'return %d;\n'% (msgsize+6))
            self.output.write(3*self.indent + '}\n\n')
 
        self.output.write(self.indent + '}; // class MspParser\n\n')
        self.output.write('} // namespace hf\n')
        self.output.close()

# Java emitter =======================================================================================

class Java_Emitter(CompileableCodeEmitter):

    def __init__(self, msgdict):

        CompileableCodeEmitter.__init__(self, 'java', 'java')

        self._copyfile('example.java', 'java/example.java')

        mkdir_if_missing('output/java/edu')
        mkdir_if_missing('output/java/edu/wlu')
        mkdir_if_missing('output/java/edu/wlu/cs')
        mkdir_if_missing('output/java/edu/wlu/cs/msppg')

        self.type2decl  = {'byte': 'byte', 'short' : 'short', 'float' : 'float', 'int' : 'int'}
        self.type2bb   = {'byte': '', 'short' : 'Short', 'float' : 'Float', 'int' : 'Int'}

        self.output = _openw('output/java/edu/wlu/cs/msppg/Parser.java')

        self._write(self._getsrc('parser-top-java'))

        # Write handler cases for incoming messages
        for msgtype in msgdict.keys():

            msgstuff = msgdict[msgtype]
            msgid = msgstuff[0]

            if msgid < 200:

                self._write(6*self.indent + 'case (byte)%d:\n' % msgid)
                self._write(7*self.indent + 'if (this.%s_handler != null) {\n' % msgtype)
                self._write(8*self.indent + 'this.%s_handler.handle_%s(\n' % (msgtype, msgtype));

                argnames = self._getargnames(msgstuff)
                argtypes = self._getargtypes(msgstuff)

                nargs = len(argnames)

                offset = 0
                for k in range(nargs):
                    argtype = argtypes[k]
                    self._write(8*self.indent + 'bb.get%s(%d)' % (self.type2bb[argtype], offset))
                    offset += self.type2size[argtype]
                    if k < nargs-1:
                        self._write(',\n')
                self._write(');\n')

                self._write(7*self.indent + '}\n')
                self._write(7*self.indent + 'break;\n\n')

        self._write(5*self.indent + '}\n' + 4*self.indent + '}\n' + 2*self.indent + '}\n' + self.indent + '}\n\n')

        for msgtype in msgdict.keys():

            msgstuff = msgdict[msgtype]
            msgid = msgstuff[0]

            argnames = self._getargnames(msgstuff)
            argtypes = self._getargtypes(msgstuff)

            # For messages from FC
            if msgid < 200:

                # Declare handler
                self._write(self.indent + 'private %s_Handler %s_handler;\n\n' % (msgtype, msgtype))
                self._write(self.indent + 
                        'public void set_%s_Handler(%s_Handler handler) {\n\n' % (msgtype, msgtype))
                self._write(2*self.indent + 'this.%s_handler = handler;\n' % msgtype)
                self._write(self.indent + '}\n\n')

                # Write serializer for requests
                self._write(self.indent + 'public byte [] serialize_%s_Request() {\n\n' % msgtype)
                paysize = self._paysize(argtypes)
                msgsize = self._msgsize(argtypes)
                self._write('\n' + 2*self.indent + 'byte [] message = new byte[6];\n\n')
                self._write(2*self.indent + 'message[0] = 36;\n')
                self._write(2*self.indent + 'message[1] = 77;\n')
                self._write(2*self.indent + 'message[2] = 60;\n')
                self._write(2*self.indent + 'message[3] = 0;\n')
                self._write(2*self.indent + 'message[4] = (byte)%d;\n' % msgid)
                self._write(2*self.indent + 'message[5] = (byte)%d;\n\n' % msgid)
                self._write(2*self.indent + 'return message;\n')
                self._write(self.indent + '}\n\n')

            # Write serializer method for messages from FC
            self._write(self.indent + 'public byte [] serialize_%s' % msgtype)
            self._write_params(self.output, argtypes, argnames)
            self._write(' {\n\n')
            paysize = self._paysize(argtypes)
            msgsize = self._msgsize(argtypes)
            self._write(2*self.indent + 'ByteBuffer bb = newByteBuffer(%d);\n\n' % paysize)
            for (argname,argtype) in zip(argnames,argtypes):
                self._write(2*self.indent + 'bb.put%s(%s);\n' % (self.type2bb[argtype], argname))
            self._write('\n' + 2*self.indent + 'byte [] message = new byte[%d];\n' % (msgsize+6))
            self._write(2*self.indent + 'message[0] = 36;\n')
            self._write(2*self.indent + 'message[1] = 77;\n')
            self._write(2*self.indent + 'message[2] = %d;\n' % (62 if msgid < 200 else 60))
            self._write(2*self.indent + 'message[3] = %d;\n' % msgsize)
            self._write(2*self.indent + 'message[4] = (byte)%d;\n' %msgdict[msgtype][0]) 
            self._write(2*self.indent + 'byte [] data = bb.array();\n')
            self._write(2*self.indent + 'int k;\n')
            self._write(2*self.indent + 'for (k=0; k<data.length; ++k) {\n')
            self._write(3*self.indent + 'message[k+5] = data[k];\n')
            self._write(2*self.indent + '}\n\n')
            self._write(2*self.indent + 'message[%d] = CRC8(message, 3, %d);\n\n' % 
                    (msgsize+5, msgsize+4))
            self._write(2*self.indent + 'return message;\n')
            self._write(self.indent + '}\n\n')

        self._write('}')

        self.output.close()

        # Write handler classes for each type of incoming message
        for msgtype in msgdict.keys():

            msgstuff = msgdict[msgtype]
            msgid = msgstuff[0]

            if msgid < 200:

                argnames = self._getargnames(msgstuff)
                argtypes = self._getargtypes(msgstuff)

                classname = '%s_Handler.java' % msgtype
                self.output = _openw('output/java/edu/wlu/cs/msppg/' + classname)
                self.output.write('/*\n%s: handler class for %s message in MSPPG\n' % (classname, msgtype))
                self._write(self._getsrc('class-top-java'))
                self.output.write('package edu.wlu.cs.msppg;\n\n')
                self.output.write('public interface %s_Handler {\n\n' % msgtype)
                self.output.write(self.indent + 'public void handle_%s' % msgtype)
                self._write_params(self.output, argtypes, argnames)
                self.output.write(';\n')
                self.output.write('}\n')

    def _write(self, s):

        self.output.write(s)

# main ===============================================================================================

if __name__ == '__main__':

    # default to input from simple example
    data = json.load(open(argv[1] if len(argv) > 1 else 'messages.json', 'r'))
 
    # takes the types of messages from the json file
    unicode_message_types = data.keys()

    # make a list of messages from the JSON file
    message_type_list = list()
    for key in unicode_message_types:
        message_type = json.dumps(key)
        clean_type = clean(message_type)
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
            argname = clean(clean(json.dumps(list(arg.keys()))))
            argtype = arg[list(arg.keys())[0]]
            if argname == 'ID':
                msgid = int(argtype)
            else:
                argtypes.append(argtype)
                argnames.append(argname)
            argument_lists.append(argnames)
        if msgid is None:
            error('Missing ID for message ' + msgtype)
        argument_types.append(argtypes)
        msgdict[msgtype] = (msgid, argnames, argtypes)

    #  make output directory if necessary
    mkdir_if_missing('output')

    # Emit Python
    Python_Emitter(msgdict)

    # Emite Java
    Java_Emitter(msgdict)

    # Emit firmware header
    HPP_Emitter(msgdict)
