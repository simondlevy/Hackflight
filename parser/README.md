# MSPPG: Multiwii Serial Protocol Parser Generator

<img src="https://github.com/simondlevy/Hackflight/blob/master/media/msppg.png" width=600>

**msppgy.py** is a standalone Python script that outputs code for parsing and generating
[MSP](http://www.armazila.com/MultiwiiSerialProtocol(draft)v02.pdf) messages
based on a simple JSON specification.  By using this script you can avoid the
lengthy and error-prone task of writing your own parsing code from scratch.
Python and Java outputs are currently supported.  (For C++ you can use the
existing [Msp](https://github.com/simondlevy/Hackflight/blob/master/src/msp.h) class.)

## Usage

Running **msppg.py** in a directory that contains a file **messages.json** will produce the following files:

* **mspparser.py**, a Python module containing a **Parser** class that you can subclass to implement your
message-handling methods

* **MspParser.java**, a Java module containing a **Parser** class that you can subclass to implement your
message-handling methods

To use the Python class, you should also install the support code for the **Parser** class:

```
% cd python
% python3 setup.py install
```

(On Unix systems you will probably have to run the setup command as sudo.)

## Example

The sample [messages.json](https://github.com/simondlevy/Hackflight/blob/master/parser/messages.json)
file specifies three message types (get normalized R/C channel values; get vehicle attitude in radians;
set motor values) that were used to generate the MSP code in the
[firmware](https://github.com/simondlevy/Hackflight/blob/master/src/task/msp/usb.h)
and
[Desktop Visualizer](https://github.com/simondlevy/Hackflight/blob/master/viz)
of the Hackflight project.

## Extending

The messages.json file currently contains just a few message specifications,
but you can easily add to it by specifying additional messages from the the MSP
[standard](http://www.armazila.com/MultiwiiSerialProtocol(draft)v02.pdf),
or add some of your own new message types.

## Caveats

The Java code produced by msppg.py has not been tested recently and may not even compile.
