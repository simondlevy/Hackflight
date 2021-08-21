{--
  Support for motors

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Motor

where

import Utils(constrain)

data Motor = Motor { motorValue :: Double }

makeMotor :: Double -> Motor

makeMotor value = Motor $ constrain value

data Motors = QuadMotors {q1 :: Motor
                        , q2 :: Motor
                        , q3 :: Motor
                        , q4 :: Motor}

getMotorValues :: Motors -> [Double]

getMotorValues (QuadMotors m1 m2 m3 m4) =

    [motorValue $ m1, motorValue $ m2, motorValue $ m3, motorValue $ m4]
