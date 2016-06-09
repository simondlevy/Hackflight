#!/usr/bin/python3

'''

msppg.py Multiwii Serial Protocol Parser Generator

Copyright (C) Rob Jones, Alec Singer, Chris Lavin, Blake Liebling, Simon D. Levy 2015

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

from sys import exit, argv, stderr
from subprocess import call
import os
import json
from pkg_resources import resource_string

def clean(string):
    cleaned_string = string[1: len(string) - 1]
    return cleaned_string

def mkdir_if_missing(dirname):
    if not os.path.exists(dirname):
        os.mkdir(dirname)

def error(errmsg):
    print(errmsg)
    exit(1)


class CodeEmitter(object):

    def __init__(self, folder, ext):

        mkdir_if_missing('output/%s' % folder)
        self._copyfile('%s.makefile' % folder, '%s/Makefile' % folder)

        self.indent = '    '

        self.type2size = {'byte': 1, 'short' : 2, 'float' : 4, 'int' : 4}

    def _copyfile(self, src, dst):

        outfile = open('output/' + dst, 'w')
        outfile.write(self._getsrc(src))
        outfile.close()

    def warning(self, cmt):

        return cmt + ' AUTO-GENERATED CODE: DO NOT EDIT!!!\n\n'

    # Helper for writing parameter list with type declarations
    def _write_params(self, outfile, argtypes, argnames, prefix = ''):

        outfile.write('(')
        outfile.write(prefix)
        for argtype,argname in zip(argtypes, argnames):
            outfile.write(self.type2decl[argtype] + ' ' +  argname)
            if argname != argnames[-1]:
                outfile.write(', ')
        outfile.write(')')


    def _paysize(self, argtypes):

        return sum([self.type2size[atype] for atype in argtypes])
    
    def _msgsize(self, argtypes):

        return self._paysize(argtypes)


    def _getsrc(self, filename):

        return resource_string('msppg_resources', filename).decode('utf-8')
 
    def _getargnames(self, message):

        return [argname for (argname,_) in self._getargs(message)]

    def _getargtypes(self, message):

        return [argtype for (_,argtype) in self._getargs(message)]

    def _getargs(self, message):

        return [(argname,argtype) for (argname,argtype) in 
                zip(message[1], message[2]) if argname.lower()!='comment']


# Python emitter ============================================================================

class Python_Emitter(CodeEmitter):

    def _copy_example(self, name):

        CodeEmitter._copyfile(self, '%s.py' % name, 'python/' + ('%s.py' % name))

    def __init__(self, msgdict):

        CodeEmitter.__init__(self, 'python', 'py')
        
        self._copy_example('getimu')
        self._copy_example('getrc')
        self._copy_example('imudisplay')
        self._copy_example('blueimudisplay')
        self._copy_example('setrc')

        mkdir_if_missing('output/python/msppg')

        self._copyfile('setup.py', 'python/setup.py')

        self.output = open('./output/python/msppg/__init__.py', 'w')

        self._write(self.warning('#'))

        self.type2pack = {'byte' : 'B', 'short' : 'h', 'float' : 'f', 'int' : 'i'}

        self._write(self._getsrc('top-py') + '\n')

        for msgtype in msgdict.keys():
            msgstuff = msgdict[msgtype]
            msgid = msgstuff[0]
            if msgid < 200:
                self._write(4*self.indent + ('if self.message_id == %d:\n\n' % msgstuff[0]))
                self._write(5*self.indent + 'if hasattr(self, \'' +  msgtype + '_Handler\'):\n\n')
                self._write(6*self.indent + 'self.%s_Handler(*struct.unpack(\'=' % msgtype)
                for argtype in self._getargtypes(msgstuff):
                    self._write('%s' % self.type2pack[argtype])
                self._write("\'" + ', self.message_buffer))\n\n')

        self._write(self._getsrc('bottom-py') + '\n')

        for msgtype in msgdict.keys():

            msgstuff = msgdict[msgtype]
            msgid = msgstuff[0]

            self._write(self.indent + 'def serialize_' + msgtype + '(self')
            for argname in self._getargnames(msgstuff):
                self._write(', ' + argname)
            self._write('):\n\n')
            self._write(self.indent*2 + 'message_buffer = struct.pack(\'')
            for argtype in self._getargtypes(msgstuff):
                self._write(self.type2pack[argtype])
            self._write('\'')
            for argname in self._getargnames(msgstuff):
                self._write(', ' + argname)
            self._write(')\n\n')
            self._write(self.indent*2)

            self._write('msg = chr(len(message_buffer)) + chr(%s) + message_buffer\n\n' % msgid)
            self._write(self.indent*2 + 'return _bytes(\'$M%c\' + msg + chr(_CRC8(msg)))\n\n' %
                    ('>' if msgid < 200 else '<'))

            if msgid < 200:

                self._write(self.indent + 'def set_%s_Handler(self, handler):\n\n' % msgtype) 
                self._write(2*self.indent + 'self.%s_Handler = handler\n\n' % msgtype)

                self._write(self.indent + 'def serialize_' + msgtype + '_Request(self):\n\n')
                self._write(2*self.indent + 'return _bytes(\'$M<\' + chr(0) + chr(%s) + chr(%s))\n\n' % 
                        (msgid, msgid))

    def _write(self, s):

        self.output.write(s)

# C++ / Arduino emitter ============================================================================

class CPP_Emitter(CodeEmitter):

    def __init__(self, msgdict):

        CodeEmitter.__init__(self, 'cpp', 'cpp')
        mkdir_if_missing('output/cpp/msppg')
        self._copyfile('example.cpp', 'cpp/example.cpp')

        mkdir_if_missing('output/arduino')
        mkdir_if_missing('output/arduino/MSPPG')
        mkdir_if_missing('output/arduino/MSPPG/examples')
        mkdir_if_missing('output/arduino/MSPPG/examples/imuexample')
        self._copyfile('imuexample.ino', 'arduino/MSPPG/examples/imuexample/imuexample.ino')

        self.type2decl = {'byte': 'byte', 'short' : 'short', 'float' : 'float', 'int' : 'int'}

        self.coutput = open('./output/cpp/msppg/msppg.cpp', 'w')
        self.houtput = open('./output/cpp/msppg/msppg.h', 'w')

        self.acoutput = open('./output/arduino/MSPPG/msppg.cpp', 'w')
        self.ahoutput = open('./output/arduino/MSPPG/msppg.h', 'w')

        self._cwrite(self.warning('//'))

        self._hwrite(self._getsrc('top-hpp'))
 
        self._cwrite('\n' + self._getsrc('top-cpp'))

        for msgtype in msgdict.keys():

            msgstuff = msgdict[msgtype]
            msgid = msgstuff[0]

            argnames = self._getargnames(msgstuff)
            argtypes = self._getargtypes(msgstuff)

            self._hwrite(self.indent*2 + 'MSP_Message serialize_%s' % msgtype)
            self._write_params(self.houtput, argtypes, argnames)
            self._write_params(self.ahoutput, argtypes, argnames)
            self._hwrite(';\n\n')

            # Write handler code for incoming messages
            if msgid < 200:

                self._cwrite(5*self.indent + ('case %s: {\n\n' % msgdict[msgtype][0]))
                nargs = len(argnames)
                offset = 0
                for k in range(nargs):
                    argname = argnames[k]
                    argtype = argtypes[k]
                    decl = self.type2decl[argtype]
                    self._cwrite(6*self.indent + decl  + ' ' + argname + ';\n')
                    self._cwrite(6*self.indent + 
                            'memcpy(&%s,  &this->message_buffer[%d], sizeof(%s));\n\n' % 
                            (argname, offset, decl))
                    offset += self.type2size[argtype]
                self._cwrite(6*self.indent + 'this->handlerFor%s->handle_%s(' % (msgtype, msgtype))
                for k in range(nargs):
                    self._cwrite(argnames[k])
                    if k < nargs-1:
                        self._cwrite(', ')
                self._cwrite(');\n')
                self._cwrite(6*self.indent + '} break;\n\n')
                
                self._hwrite(self.indent*2 + 'MSP_Message serialize_%s_Request();\n\n' % msgtype)
                self._hwrite(self.indent*2 + 
                        'void set_%s_Handler(class %s_Handler * handler);\n\n' % (msgtype, msgtype))

        self._hwrite(self.indent + 'private:\n\n')

        for msgtype in msgdict.keys():

            msgstuff = msgdict[msgtype]
            msgid = msgstuff[0]
            
            if msgid < 200:
                self._hwrite(2*self.indent + 
                        'class %s_Handler * handlerFor%s;\n\n' % (msgtype, msgtype));

        self._hwrite('};\n');

        self._cwrite(self._getsrc('bottom-cpp'))
 
        for msgtype in msgdict.keys():

            msgstuff = msgdict[msgtype]
            msgid = msgstuff[0]

            argnames = self._getargnames(msgstuff)
            argtypes = self._getargtypes(msgstuff)

            # Incoming messages
            if msgid < 200:

                # Declare handler class
                self._hwrite('\n\n' + 'class %s_Handler {\n' % msgtype)
                self._hwrite('\n' + self.indent + 'public:\n\n')
                self._hwrite(2*self.indent + '%s_Handler() {}\n\n' % msgtype)
                self._hwrite(2*self.indent + 'virtual void handle_%s' % msgtype)
                self._write_params(self.houtput, argtypes, argnames)
                self._write_params(self.ahoutput, argtypes, argnames)
                self._hwrite('{ }\n\n')
                self._hwrite('};\n\n')
                
                # Write handler method
                self._cwrite('void MSP_Parser::set_%s_Handler(class %s_Handler * handler) {\n\n' %
                        (msgtype, msgtype))
                self._cwrite(self.indent + 'this->handlerFor%s = handler;\n' % msgtype)
                self._cwrite('}\n\n')

                # Write request method
                self._cwrite('MSP_Message MSP_Parser::serialize_%s_Request() {\n\n' % msgtype)
                self._cwrite(self.indent + 'MSP_Message msg;\n\n')
                self._cwrite(self.indent + 'msg.bytes[0] = 36;\n')
                self._cwrite(self.indent + 'msg.bytes[1] = 77;\n')
                self._cwrite(self.indent + 'msg.bytes[2] = %d;\n' % 60 if msgid < 200 else 62)
                self._cwrite(self.indent + 'msg.bytes[3] = 0;\n')
                self._cwrite(self.indent + 'msg.bytes[4] = %d;\n' % msgid)
                self._cwrite(self.indent + 'msg.bytes[5] = %d;\n\n' % msgid)
                self._cwrite(self.indent + 'msg.len = 6;\n\n')
                self._cwrite(self.indent + 'return msg;\n')
                self._cwrite('}')


            # Add parser method for serializing message
            self._cwrite('MSP_Message MSP_Parser::serialize_%s' % msgtype)
            self._write_params(self.coutput, argtypes, argnames)
            self._write_params(self.acoutput, argtypes, argnames)
            self._cwrite(' {\n\n')
            self._cwrite(self.indent + 'MSP_Message msg;\n\n')
            msgsize = self._msgsize(argtypes)
            self._cwrite(self.indent + 'msg.bytes[0] = 36;\n')
            self._cwrite(self.indent + 'msg.bytes[1] = 77;\n')
            self._cwrite(self.indent + 'msg.bytes[2] = 62;\n')
            self._cwrite(self.indent + 'msg.bytes[3] = %d;\n' % msgsize)
            self._cwrite(self.indent + 'msg.bytes[4] = %d;\n\n' % msgid)
            nargs = len(argnames)
            offset = 5
            for k in range(nargs):
                argname = argnames[k]
                argtype = argtypes[k]
                decl = self.type2decl[argtype]
                self._cwrite(self.indent + 
                        'memcpy(&msg.bytes[%d], &%s, sizeof(%s));\n' %  (offset, argname, decl))
                offset += self.type2size[argtype]
            self._cwrite('\n')
            self._cwrite(self.indent + 
                    'msg.bytes[%d] = CRC8(&msg.bytes[3], %d);\n\n' % (msgsize+5, msgsize+2))
            self._cwrite(self.indent + 'msg.len = %d;\n\n' % (msgsize+6))
            self._cwrite(self.indent + 'return msg;\n')
            self._cwrite('}\n\n')
 
    def _cwrite(self, s):

        self.coutput.write(s)
        self.acoutput.write(s)

    def _hwrite(self, s):

        self.houtput.write(s)
        self.ahoutput.write(s)

# C emitter ===============================================================================

class C_Emitter(CodeEmitter):

    def __init__(self, msgdict):

        CodeEmitter.__init__(self, 'c', 'c')
        mkdir_if_missing('output/c/msppg')
        self._copyfile('example.c', 'c/example.c')

        self.type2decl = {'byte': 'char', 'short' : 'short', 'float' : 'float', 'int' : 'int'}

        self.coutput = open('./output/c/msppg/msppg.c', 'w')
        self.houtput = open('./output/c/msppg/msppg.h', 'w')

        self._cwrite(self.warning('//'))

        self._hwrite(self._getsrc('top-h'))

        for msgtype in msgdict.keys():

            msgstuff = msgdict[msgtype]
            msgid = msgstuff[0]

            if msgid < 200:

                argnames = self._getargnames(msgstuff)
                argtypes = self._getargtypes(msgstuff)

                self._hwrite(self.indent + 'void (*handler_for_%s)' % msgtype)
                self._write_params(self.houtput, argtypes, argnames)
                self._hwrite(';\n')

        self._hwrite(self._getsrc('bottom-h'))
 
        self._cwrite('\n' + self._getsrc('top-c'))

        for msgtype in msgdict.keys():

            msgstuff = msgdict[msgtype]
            msgid = msgstuff[0]

            argnames = self._getargnames(msgstuff)
            argtypes = self._getargtypes(msgstuff)

            self._hwrite('msp_message_t msp_serialize_%s' % msgtype)
            self._write_params(self.houtput, argtypes, argnames)
            self._hwrite(';\n\n')

            # Write handler code for incoming messages
            if msgid < 200:

                self._cwrite(5*self.indent + ('case %s: {\n\n' % msgdict[msgtype][0]))
                nargs = len(argnames)
                offset = 0
                for k in range(nargs):
                    argname = argnames[k]
                    argtype = argtypes[k]
                    decl = self.type2decl[argtype]
                    self._cwrite(6*self.indent + decl  + ' ' + argname + ';\n')
                    self._cwrite(6*self.indent + 
                            'memcpy(&%s,  &parser->message_buffer[%d], sizeof(%s));\n\n' % 
                            (argname, offset, decl))
                    offset += self.type2size[argtype]
                self._cwrite(6*self.indent + 'parser->handler_for_%s(' % msgtype)
                for k in range(nargs):
                    self._cwrite(argnames[k])
                    if k < nargs-1:
                        self._cwrite(', ')
                self._cwrite(');\n')
                self._cwrite(6*self.indent + '} break;\n\n')
                
                self._hwrite('msp_message_t msp_serialize_%s_request();\n\n' % msgtype)
                self._hwrite('void msp_set_%s_handler(msp_parser_t * parser, void (*handler)' % msgtype)
                self._write_params(self.houtput, argtypes, argnames)
                self._hwrite(');\n\n')

        self._cwrite(self._getsrc('bottom-cpp'))
 
        for msgtype in msgdict.keys():

            msgstuff = msgdict[msgtype]
            msgid = msgstuff[0]

            argnames = self._getargnames(msgstuff)
            argtypes = self._getargtypes(msgstuff)

            # Incoming messages
            if msgid < 200:

                # Write handler method
                self._cwrite('void msp_set_%s_handler(msp_parser_t * parser, void (*handler)' % msgtype)
                self._write_params(self.coutput, argtypes, argnames)
                self._cwrite(') {\n\n')
                self._cwrite(self.indent + 'parser->handler_for_%s = handler;\n' % msgtype)
                self._cwrite('}\n\n')

                # Write request method
                self._cwrite('msp_message_t serialize_%s_Request() {\n\n' % msgtype)
                self._cwrite(self.indent + 'msp_message_t msg;\n\n')
                self._cwrite(self.indent + 'msg.bytes[0] = 36;\n')
                self._cwrite(self.indent + 'msg.bytes[1] = 77;\n')
                self._cwrite(self.indent + 'msg.bytes[2] = %d;\n' % 60 if msgid < 200 else 62)
                self._cwrite(self.indent + 'msg.bytes[3] = 0;\n')
                self._cwrite(self.indent + 'msg.bytes[4] = %d;\n' % msgid)
                self._cwrite(self.indent + 'msg.bytes[5] = %d;\n\n' % msgid)
                self._cwrite(self.indent + 'msg.len = 6;\n\n')
                self._cwrite(self.indent + 'return msg;\n')
                self._cwrite('}\n\n')

            # Add parser method for serializing message
            self._cwrite('msp_message_t msp_serialize_%s' % msgtype)
            self._write_params(self.coutput, argtypes, argnames)
            self._cwrite(' {\n\n')
            self._cwrite(self.indent + 'msp_message_t msg;\n\n')
            msgsize = self._msgsize(argtypes)
            self._cwrite(self.indent + 'msg.bytes[0] = 36;\n')
            self._cwrite(self.indent + 'msg.bytes[1] = 77;\n')
            self._cwrite(self.indent + 'msg.bytes[2] = 62;\n')
            self._cwrite(self.indent + 'msg.bytes[3] = %d;\n' % msgsize)
            self._cwrite(self.indent + 'msg.bytes[4] = %d;\n\n' % msgid)
            nargs = len(argnames)
            offset = 5
            for k in range(nargs):
                argname = argnames[k]
                argtype = argtypes[k]
                decl = self.type2decl[argtype]
                self._cwrite(self.indent + 
                        'memcpy(&msg.bytes[%d], &%s, sizeof(%s));\n' %  (offset, argname, decl))
                offset += self.type2size[argtype]
            self._cwrite('\n')
            self._cwrite(self.indent + 
                    'msg.bytes[%d] = CRC8(&msg.bytes[3], %d);\n\n' % (msgsize+5, msgsize+2))
            self._cwrite(self.indent + 'msg.len = %d;\n\n' % (msgsize+6))
            self._cwrite(self.indent + 'return msg;\n')
            self._cwrite('}\n\n')
 
    def _cwrite(self, s):

        self.coutput.write(s)

    def _hwrite(self, s):

        self.houtput.write(s)

# Java emitter =======================================================================================

class Java_Emitter(CodeEmitter):

    def __init__(self, msgdict):

        CodeEmitter.__init__(self, 'java', 'java')

        self._copyfile('example.java', 'java/example.java')

        mkdir_if_missing('output/java/edu')
        mkdir_if_missing('output/java/edu/wlu')
        mkdir_if_missing('output/java/edu/wlu/cs')
        mkdir_if_missing('output/java/edu/wlu/cs/msppg')

        self.type2decl  = {'byte': 'byte', 'short' : 'short', 'float' : 'float', 'int' : 'int'}
        self.type2bb   = {'byte': '', 'short' : 'Short', 'float' : 'Float', 'int' : 'Int'}

        self.output = open('./output/java/edu/wlu/cs/msppg/Parser.java', 'w')

        self._write(self.warning('//'))

        self._write(self._getsrc('top-java'))

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

        self._write(self._getsrc('bottom-java'))

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

                self.output = open('./output/java/edu/wlu/cs/msppg/%s_Handler.java' % msgtype, 'w')
                self.output.write(self.warning('//'))
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
    data = json.load(open(argv[1] if len(argv) > 1 else 'example.json'))
 
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

    # Emit C++
    CPP_Emitter(msgdict)

    # Emit C
    C_Emitter(msgdict)

    # Emite Java
    Java_Emitter(msgdict)
