{--
  Micrcontroller pin

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Pin where

import Language.Copilot

data Pin = Pin { pin :: Stream Word8 }


