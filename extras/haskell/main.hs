{--
  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

import Hackflight

main :: IO ()

main = let h = Hackflight 0
       in putStrLn (run h)
