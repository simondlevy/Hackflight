{--
  Socket-based multicopter control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Server (runServer) where

import Control.Applicative
import Network.Socket
import Network.Socket.ByteString -- from network
import Data.ByteString.Internal
import Data.Either.Utils -- from MissingH
import Data.Serialize -- from cereal

import Demands
import Mixer
import State
import PidControl(PidFun, PidState)

type RunPids = Demands -> Time -> VehicleState -> (PidFun, PidState) -> Mixer 
               -> (Motors, (PidFun, PidState))

runPids :: RunPids
runPids demands time vstate controller mixer = 

  -- Run the PID controller to get new demands and controller state
  let (newDemands, newControllerState) = (fst controller) time vstate demands (snd controller)

  -- Run the mixer on the new demands to get the motors, then return them and the new
  -- controller state
  in ((mixer newDemands), ((fst controller), newControllerState))


runServer :: (PidFun, PidState) -> Mixer -> IO ()
runServer controller mixer = withSocketsDo $

    do 

       (telemetryServerSocket, telemetryServerSocketAddress) <- makeUdpSocket "5001"

       (motorClientSocket, motorClientSocketAddress) <- makeUdpSocket "5000"

       bind telemetryServerSocket telemetryServerSocketAddress

       putStrLn "Hit the Play button ..."

       loop telemetryServerSocket motorClientSocket motorClientSocketAddress controller 

    where loop telemetryServerSocket motorClientSocket motorClientSockAddr pidController  =

              do 

                  -- Get raw bytes for time, 12D state vector, and stick demands from
                  -- client (sim)
                  (msgIn, _) <- Network.Socket.ByteString.recvFrom telemetryServerSocket 136

                  -- Convert bytes to a list of doubles
                  let d = bytesToDoubles msgIn

                  -- Parse the doubles into timed, vehicle state, and stick demands
                  let time = d!!0
                  let vehicleState = VehicleState (d!!1) (d!!2) (d!!3) (d!!4) (d!!5) (d!!6)
                                                  (d!!7) (d!!8) (d!!9) (d!!10) (d!!11) (d!!12)
                  let stickDemands = Demands (d!!13) (d!!14) (d!!15) (d!!16)

                  -- Run the mixer on the demands to get the motor values
                  let (motors, newPidController)  = runPids stickDemands
                                                            time
                                                            vehicleState
                                                            pidController
                                                            mixer

                  -- Send the motor values to the client
                  _ <- Network.Socket.ByteString.sendTo
                        motorClientSocket
                        (doublesToBytes [(m1 motors), (m2 motors), (m3 motors), (m4 motors)])
                        motorClientSockAddr

                  -- Repeat until user presses top button
                  if time > 0 then
                      loop telemetryServerSocket
                           motorClientSocket
                           motorClientSockAddr
                           newPidController
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
