{--
  Socket-based multicopter control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Server (runServer) where

import Network.Socket
import Network.Socket.ByteString -- from network

import Sockets(makeUdpSocket)
import Utils(bytesToDoubles, doublesToBytes)

import Hackflight(HackflightFun)
import Receiver
import Sensor
import PidControllers(PidController)
import Mixer
import Motor(getMotorValues)

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
            pidControllers
            mixer

loop :: Socket ->
        Socket ->
        SockAddr ->
        HackflightFun ->
        [PidController] ->
        Mixer ->
        IO ()

loop telemetryServerSocket
     motorClientSocket
     motorClientSockAddr
     hackflightFun
     pidControllers
     mixer =

  do 

      -- Get raw bytes for time and 12D state vector from client
      (telemBytes, _) <- 
          Network.Socket.ByteString.recvFrom telemetryServerSocket (8*17)

      -- Convert bytes to a list of doubles
      let telem = bytesToDoubles telemBytes

      -- First value is time, which will be negative when user quits
      if telem!!0 >= 0 

      then do

          print $ telem!!0

          -- Simulate a sensor using the next 12 values (state)
          let sensor = SimSensor (telem!!1) (telem!!2) (telem!!3) 
                                 (telem!!4) (telem!!5) (telem!!6) 
                                 (telem!!7) (telem!!8) (telem!!9) 
                                 (telem!!10) (telem!!11) (telem!!12) 

          -- Simulate a receiver using the final four values (stick demands)
          let receiver = SimReceiver (telem!!13) (telem!!14)
                                     (telem!!15) (telem!!16)

          -- Run the Hackflight algorithm to get the motor values
          -- and updated PID controllers
          let (motors, newPidControllers) = hackflightFun receiver
                                                          [sensor]
                                                          pidControllers
                                                          mixer
          
          -- Send the motor values to the client
          _ <- Network.Socket.ByteString.sendTo
                motorClientSocket
                (doublesToBytes (getMotorValues motors))
                motorClientSockAddr

          loop telemetryServerSocket
               motorClientSocket
               motorClientSockAddr
               hackflightFun
               newPidControllers
               mixer

        else putStrLn "Done"
