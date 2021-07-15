{--
  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

import Hackflight
import Board
import Receiver
import Mixer

main :: IO ()

main = let h = Hackflight Board Receiver (Mixer 0)
       in putStrLn (show (run h))
