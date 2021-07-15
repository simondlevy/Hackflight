{--
  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

import Hackflight
import Board
import Receiver

main :: IO ()

main = let h = Hackflight Board Receiver
       in putStrLn (show (run h))
