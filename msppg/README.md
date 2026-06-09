# MSPPG: Multiwii Serial Protocol Parser Generator

<img src="https://github.com/simondlevy/Hackflight/blob/master/media/msppg.png" width=600>

**msppgy.py** is a standalone Python script that outputs code for parsing and generating
[MSP](https://grokipedia.com/page/MultiWii_Serial_Protocol) messages
based on a simple JSON specification.  By using this script you can avoid the
lengthy and error-prone task of writing your own parsing code from scratch.
Python and Java outputs are currently supported. 
## Usage

Running **msppg.py** in a directory that contains a file **messages.json** will produce the following files:

* **mspp.py**, a Python module containing a **Parser** class that you can subclass to implement your
message-handling methods

* **MspParser.java**, a Java module containing a **Parser** class that you can subclass to implement your
message-handling methods

For C++ you can use the
existing [MspParser](https://github.com/simondlevy/Hackflight/blob/master/src/firmware/msp/parser.hpp)
and [MspSerializer](https://github.com/simondlevy/Hackflight/blob/master/src/firmware/msp/serializer.hpp)
classes.

## Example

The sample [messages.json](messages.json) file specifies the messages used in Hackflight.

## Caveats

The Java code produced by msppg.py has not been tested recently and may not even compile.
