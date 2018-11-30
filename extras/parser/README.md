# MSPPG
Multiwii Serial Protocol Parser Generator

Steps:

1. Run <b>msppg.py</b>

2. Change directories to the language of your choice (<b>output/python</b> or <b>output/java</b>)

3. Python users, Linux: <tt>sudo python3 setup.py install</tt>

4. Python users, Windows: <tt>python3 setup.py install</tt>

5. Java users, Linux: run <tt>make jar</tt> to build <b>msppg.jar</b>, which
   can then be used as a library for Android projects and other Java-based
   work.

6. Java users, Windows: use your favorite Java IDE (Netbeans, Eclipse) to build <b>msppg.jar</b>

<b>Extending</b>

The messages.json file currently contains just a few message specifications,
but you can easily add to it by specifying additional messages from the the MSP
[standard](http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol),
or add some of your own new message types.  MSPPG currently supports types
byte, short, int, and float, but we will likely add int as the need arises.
