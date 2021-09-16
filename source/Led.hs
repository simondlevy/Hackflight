{--
  LED

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Led where

import Language.Copilot

data Led = Led { pin :: Stream Word8 }


