{--
  Socket-based "sensor"

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module SimSensor (simSensorClosure) where

import VehicleState
import Sensor
import Sockets(makeUdpSocket)


import Network.Socket
import Network.Socket.ByteString -- from network

getNewVehicleState :: Socket ->  VehicleState -> IO (Double)
getNewVehicleState telemetrySocket vehicleState =
  do
    (msgIn, _) <- Network.Socket.ByteString.recvFrom telemetrySocket 136
    return 3.5


simSensorClosure :: Socket -> Sensor
simSensorClosure telemetrySocket = 

    \vehicleState -> vehicleState


makeSimSensorClosure = 

    do 

       (telemetrySocket, telemetrySocketAddress) <-
           makeUdpSocket "5001"

       bind telemetrySocket telemetrySocketAddress

       return (simSensorClosure telemetrySocket)
{--

import VehicleState
import Control.Applicative
import Network.Socket
import Data.ByteString.Internal
import Data.Either.Utils -- from MissingH
import Data.Serialize -- from cereal

runServer :: HackflightFun -> [PidController] -> Mixer -> IO ()

runServer hackflight pidControllers mixer = withSocketsDo $

    do 

       (telemetrySocket, telemetrySocketAddress) <-
           makeUdpSocket "5001"

       bind telemetrySocket telemetrySocketAddress

       putStrLn "Hit the Play button ..."

       loop telemetrySocket
            motorClientSocket
            motorClientSocketAddress
            hackflight
            mixer
            pidControllers

loop :: Socket ->
        Socket ->
        SockAddr ->
        HackflightFun ->
        Mixer ->
        [PidController]->
        IO ()

loop telemetrySocket
     motorClientSocket
     motorClientSockAddr
     hackflight
     mixer
     pidControllers =

  do 

      -- Get raw bytes for time, 12D state vector, and stick
      -- demands from client (sim)
      (msgIn, _) <- 
          Network.Socket.ByteString.recvFrom telemetrySocket 136

      -- Convert bytes to a list of doubles
      let d = bytesToDoubles msgIn

      -- Parse the doubles into timed, vehicle state, and stick
      -- demands
      let time = d!!0

      let vehicleState = VehicleState (d!!1) (d!!2) (d!!3) (d!!4)
                                      (d!!5) (d!!6) (d!!7) (d!!8)
                                      (d!!9) (d!!10) (d!!11) (d!!12)

      let stickDemands = Demands (d!!13) (d!!14) (d!!15) (d!!16)

      -- Run the Hackflight algorithm to get the motor values
      let (motors, newPidControllers) = hackflight stickDemands
                                                      vehicleState
                                                      mixer
                                                      pidControllers
      -- Send the motor values to the client
      _ <- Network.Socket.ByteString.sendTo
            motorClientSocket
            (doublesToBytes (motorValues motors))
            motorClientSockAddr

      -- Repeat until user presses stop button in simulator
      if time >= 0  then
          loop telemetrySocket
               motorClientSocket
               motorClientSockAddr
               hackflight
               mixer
               newPidControllers
      else
          putStrLn "Done"

-- http://book.realworldhaskell.org/read/sockets-and-syslog.html

makeUdpSocket :: String -> IO (Socket, SockAddr)
makeUdpSocket port =
    do 
       addrInfo <- getAddrInfo
                   (Just (defaultHints {addrFlags = [AI_PASSIVE]}))
                   Nothing
                   (Just port)
       let addr = head addrInfo
       sock <- socket (addrFamily addr) Datagram defaultProtocol
       return (sock, (addrAddress addr))

-- https://stackoverflow.com/questions/20912582/haskell-bytestring-to-float-array

doublesToBytes :: [Double] -> ByteString
doublesToBytes = runPut . mapM_ putFloat64le

bytesToDoubles :: ByteString -> [Double]
bytesToDoubles bs = (fromRight ((runGet $ many getFloat64le) bs))

--}
