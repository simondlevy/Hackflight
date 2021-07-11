'''
  Socket-based multicopter class

  Copyright(C) 2021 Simon D.Levy

  MIT License
'''

from threading import Thread
import socket
import numpy as np
import sys
import time
import cv2


def _run_telemetry(obj,
                   host,
                   motor_port,
                   telemetryServerSocket,
                   motorClientSocket):

    running = False

    while True:

        try:
            data, _ = telemetryServerSocket.recvfrom(8*13)
        except Exception:
            obj.done = True
            break

        telemetryServerSocket.settimeout(.1)

        telem = np.frombuffer(data)

        if not running:
            MulticopterServer.debug('Running')
            running = True

        if telem[0] < 0:
            motorClientSocket.close()
            telemetryServerSocket.close()
            break

        motorvals = obj.getMotors(telem[0], telem[1:])

        motorClientSocket.sendto(np.ndarray.tobytes(motorvals),
                                 (host, motor_port))

        time.sleep(.001)

class MulticopterServer(object):

    # See Bouabdallah (2004)
    (STATE_X,
     STATE_DX,
     STATE_Y,
     STATE_DY,
     STATE_Z,
     STATE_DZ,
     STATE_PHI,
     STATE_DPHI,
     STATE_THETA,
     STATE_DTHETA,
     STATE_PSI,
     STATE_DPSI) = range(12)

    def start(self,
              host='127.0.0.1',
              motor_port=5000,
              telem_port=5001,
              image_port=5002,
              image_rows=480,
              image_cols=640):

        self.telem = None
        self.done = False

        # Telemetry in and motors out run on their own thread
        motorClientSocket = MulticopterServer._make_udpsocket()
        telemetryServerSocket = MulticopterServer._make_udpsocket()
        telemetryServerSocket.bind((host, telem_port))

        MulticopterServer.debug('Hit the Play button ...')

        telemetryThread = Thread(target=_run_telemetry,
                                 args=(self,
                                       host,
                                       motor_port,
                                       telemetryServerSocket,
                                       motorClientSocket))


        # Serve a socket with a maximum of one client
        imageServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        imageServerSocket.bind((host, image_port))
        imageServerSocket.listen(1)

        # This will block (wait) until a client connets
        imageConn, _ = imageServerSocket.accept()
        imageConn.settimeout(1)
        MulticopterServer.debug('Got a connection!')

        telemetryThread.start()

        while not self.done:

            try:
                imgbytes = imageConn.recv(image_rows*image_cols*4)

            except Exception:  # likely a timeout from sim quitting
                break

            if len(imgbytes) == image_rows*image_cols*4:

                rgba_image = np.reshape(np.frombuffer(imgbytes, 'uint8'),
                                        (image_rows, image_cols, 4))

                image = cv2.cvtColor(rgba_image, cv2.COLOR_RGBA2RGB)

                self.handleImage(image)

    def handleImage(self, image):
        '''
        Override for your application
        '''
        cv2.imshow('Image', image)
        cv2.waitKey(1)

    def getMotors(self, time, state):
        '''
        Override for your application
        '''
        return np.array([0.6, 0.6, 0.6, 0.6])

    @staticmethod
    def debug(msg):
        print(msg)
        sys.stdout.flush()

    @staticmethod
    def _make_udpsocket():

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)

        return sock
