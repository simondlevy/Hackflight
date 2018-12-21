#!/usr/bin/env python3

import bluetooth
from time import sleep

bd_addr = '00:06:66:73:E3:E8'

port = 1

sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
sock.connect((bd_addr, port))

print('connected to ' + bd_addr)

while True:

    #print(sock.recv(10).decode())

    sleep(1)

    print('okay')

sock.close()
