{--
  Socket-based multicopter control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Server (run) where

import Control.Applicative
import Network.Socket
import Network.Socket.ByteString -- from network
import Data.ByteString.Internal
import Data.Either.Utils -- from MissingH
import Data.Serialize -- from cereal

import Mixer
import State
import AltHoldPid

run :: AltHoldController -> Mixer -> IO ()
run controller mixer = withSocketsDo $

    do 

       (telemetryServerSocket, telemetryServerSocketAddress) <- makeUdpSocket "5001"

       (motorClientSocket, motorClientSocketAddress) <- makeUdpSocket "5000"

       bind telemetryServerSocket telemetryServerSocketAddress

       putStrLn "Hit the Play button ..."

       loop telemetryServerSocket motorClientSocket motorClientSocketAddress controller 

    where loop telemetryServerSocket motorClientSocket motorClientSockAddr pidController  =

              do 

                  -- Get raw bytes for time and 12D state vector from client (sim)
                  (msgIn, _) <- Network.Socket.ByteString.recvFrom telemetryServerSocket 104

                  -- Convert bytes to a list of doubles
                  let v = bytesToDoubles msgIn

                  -- Parse the doubles into time and vehicle state
                  let t = head v
                  let vs = makeState (tail v)

                  let controllerFun = fun pidController

                  -- Run the PID controller to get new demands
                  let (demands, newControllerState) = controllerFun t vs  demands (state pidController)

                  -- Run the mixer on the demands to get the motor values
                  let motors = mixer demands

                  -- Send the motor values to the client
                  _ <- Network.Socket.ByteString.sendTo
                        motorClientSocket
                        (doublesToBytes [(m1 motors), (m2 motors), (m3 motors), (m4 motors)])
                        motorClientSockAddr

                  -- Repeat
                  loop telemetryServerSocket
                       motorClientSocket
                       motorClientSockAddr
                       (AltHoldController controllerFun newControllerState )

-- http://book.realworldhaskell.org/read/sockets-and-syslog.html

makeUdpSocket :: String -> IO (Socket, SockAddr)
makeUdpSocket port =
    do 
       addrInfo <- getAddrInfo (Just (defaultHints {addrFlags = [AI_PASSIVE]})) Nothing (Just port)
       let addr = head addrInfo
       sock <- socket (addrFamily addr) Datagram defaultProtocol
       return (sock, (addrAddress addr))

-- https://stackoverflow.com/questions/20912582/haskell-bytestring-to-float-array

doublesToBytes :: [Double] -> ByteString
doublesToBytes = runPut . mapM_ putFloat64le

bytesToDoubles :: ByteString -> [Double]
bytesToDoubles bs = (fromRight ((runGet $ many getFloat64le) bs))
