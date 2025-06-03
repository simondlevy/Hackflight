'''
Copyright (C) 2025 Simon D. Levy

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, in version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
'''

import socket
import sys
import time


#RPI_ADDRESS = 'B8:27:EB:E0:1D:07'  # Onboard
RPI_ADDRESS = 'B8:27:EB:3F:AB:47' # PiHat

def connect_to_server(port):

    while True:

        try:

            print('Connecting to server %s:%d ... ' % (RPI_ADDRESS, port), end='')
            sys.stdout.flush()

            # Create a Bluetooth or IP socket depending on address format
            client = (socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                      socket.BTPROTO_RFCOMM)
                      if ':' in RPI_ADDRESS
                      else socket.socket(socket.AF_INET, socket.SOCK_STREAM))

            try:
                client.connect((RPI_ADDRESS, port))
                print(' connected')
                break

            except Exception as e:
                print(str(e) + ': is server running?')
                time.sleep(1)

        except KeyboardInterrupt:
            break

    return client
