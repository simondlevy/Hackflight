# MSPPG
Multiwii Serial Protocol Parser Generator for Python3, Java, and C++, and C.

Instructions 

The script msppg.py is ready to-run using your favorite Python interpreter: command-line, IDLE, etc.  To install so you can run it anywhere, do

% python3 setup.py install

from a Windows command shell, or

% sudo python3 setup.py install

in Unix (Linux, OS X).

Once the package is installed, you can put your example.json file anywhere and run the following:

% msppg.py

which will create output/python, output/java/, output/c, output/cpp, and output/arduino. If you're on a Unix system
(Linux, Mac OS X), you can then cd to one of the first four output directories and do

% make test

to test the code.  On Windows, the easiest way to test would be to modify the Python scripts to use 'COM3' or
another port to access your flight controller.

In output/python you can also run the msp-imudisplay.py program, which uses Tkinter and NumPy to visualize the Attitude messages coming from a flight controller (tested with AcroNaze running Baseflight).  In output/java you can do

% make jar

to build the msppg.jar file, which can then be used as a library for Android projects and other Java-based work.

The Arduino example allows you to control the pitch of a buzzer using the pitch from the IMU. You should use an Arduino Mega or other Arduino that has TX1/RX1 pins, and make the following connections:

(1) Naze - to Arduino ground.
(2) Naze + to Arduino 5v.
(3) Naze TX to Arduino RX1.
(4) Naze RX to Arduino TX1.
(5) Buzzer + to Arduino digitial 8.
(6) Buzzer - to Arduino ground.

Then copy the output/arduino/MSPPG folder into your Arduino libaries folder, launch the Arduino IDE, and find the MSPPG submenu under the File/Examples menu.

The msp-example.json file currently contains just a few message specifications, but you can easily add to it by specifying additional messages from the MSP: http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol. 
MSPPG currently supports types byte, short, and float, but we will likely add int as the need arises.
