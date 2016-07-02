'''
socket_server.py - Python code for serving a socket on an IP address

    Copyright (C) 2014 Simon D. Levy

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as 
    published by the Free Software Foundation, either version 3 of the 
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
'''


import socket
import time

def serve_socket(port, host='localhost'):
    '''
    Serves a blocking socket on the specified port.  Returns a new socket object 
    representing the client, on which send() and recv() can be invoked.
    '''

    print('Server: running on port %d' % port)
    
    sock = None   
        
    while True:
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
 
        sock.settimeout(1)

        try:
            sock.bind((host, port))
            break
            
        except socket.error as err:
            print('Server: bind failed: ' + str(err))
            exit(1)
            
        time.sleep(1)
            
    sock.listen(1) 
    
    print('Server: waiting for client to connect ...')
    try:
        client, _ = sock.accept()
        print('Server: accepted connection')
       
        
    except:
        print('Failed')
        exit(1)

    return client
