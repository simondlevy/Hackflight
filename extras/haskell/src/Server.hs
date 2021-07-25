{--
  Socket-based multicopter control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Server (runServer) where

import Network.Socket
import Network.Socket.ByteString -- from network

import Utils(bytesToDoubles, doublesToBytes, slice)
import Sockets(makeUdpSocket)
import SimReceiver(simReceiver)
import SimSensor(simSensorClosure)
import Mixer
import Hackflight(HackflightFun)
import PidControl(PidController)

runServer :: HackflightFun -> [PidController] -> Mixer -> IO ()

-- runServer hackflight pidControllers mixer = withSocketsDo $
runServer hackflightFun pidControllers mixer = 

    do 

       (telemetryServerSocket, telemetryServerSocketAddress) <-
           makeUdpSocket "5001"

       (motorClientSocket, motorClientSocketAddress) <- makeUdpSocket "5000"

       bind telemetryServerSocket telemetryServerSocketAddress
       putStrLn "Hit the Play button ..."

       loop telemetryServerSocket
            motorClientSocket
            motorClientSocketAddress
            hackflightFun
            mixer
            pidControllers

loop :: Socket ->
        Socket ->
        SockAddr ->
        HackflightFun ->
        Mixer ->
        [PidController] ->
        IO ()

loop telemetryServerSocket
     motorClientSocket
     motorClientSockAddr
     hackflightFun
     mixer
     pidControllers =

  do 

      -- Get raw bytes for time and 12D state vector from client
      (telemBytes, _) <- 
          Network.Socket.ByteString.recvFrom telemetryServerSocket (8*17)

      -- Convert bytes to a list of doubles
      let telem = bytesToDoubles telemBytes

      -- First value is time, which will be negative when user quits
      if telem!!0 >= 0 

      then do

          let sensor = simSensorClosure $ slice telem 1 12

          let receiver = simReceiver $ slice telem 13 16

          -- Run the Hackflight algorithm to get the motor values
          let (motors, newPidControllers) = hackflightFun receiver
                                                          [sensor]
                                                          mixer
                                                          pidControllers
          -- Send the motor values to the client
          _ <- Network.Socket.ByteString.sendTo
                motorClientSocket
                (doublesToBytes (motorValues motors))
                motorClientSockAddr

          loop telemetryServerSocket
               motorClientSocket
               motorClientSockAddr
               hackflightFun
               mixer
               newPidControllers

        else putStrLn "Done"
