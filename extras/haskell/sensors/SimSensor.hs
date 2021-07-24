{--
  Socket-based "sensor"

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module SimSensor (simSensorClosure) where

import VehicleState
import Sensor
import Sockets(makeUdpSocket)
import Utils(bytesToDoubles)

import Network.Socket
import Network.Socket.ByteString -- from network

getNewVehicleState :: Socket ->  VehicleState -> IO VehicleState

getNewVehicleState telemetrySocket _oldVehicleState =

    do

        -- Get raw bytes for time and 12D state vector from client
        (telemBytes, _) <- 
            Network.Socket.ByteString.recvFrom telemetrySocket 104

        -- Convert bytes to a list of doubles
        let telem = bytesToDoubles telemBytes

        let newVehicleState = VehicleState (telem!!1) (telem!!2) (telem!!3)
                                           (telem!!4) (telem!!5) (telem!!6)
                                           (telem!!7) (telem!!8) (telem!!9)
                                           (telem!!10) (telem!!11) (telem!!12)

        return newVehicleState 

simSensorClosure :: Socket -> Sensor
simSensorClosure telemetrySocket = 

    \vehicleState -> vehicleState


makeSimSensorClosure = 

    do 

       (telemetrySocket, telemetrySocketAddress) <-
           makeUdpSocket "5001"

       bind telemetrySocket telemetrySocketAddress

       return (simSensorClosure telemetrySocket)
