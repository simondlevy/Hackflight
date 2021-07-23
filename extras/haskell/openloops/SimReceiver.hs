{--
  "Receiver" that gets its data from simulator

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module SimReceiver

where

import Control.Applicative
import Network.Socket
import Network.Socket.ByteString -- from network
import Data.ByteString.Internal
import Data.Either.Utils -- from MissingH
import Data.Serialize -- from cereal

import Demands
import OpenLoopControl

simReceiver :: OpenLoopController

simReceiver = Demands 0 0 0 0
