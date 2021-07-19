{--
  Socket-based multicopter control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Server (run) where

import State
import PidControl(PidFun, PidState)

run :: (PidFun, PidState) -> IO ()
run controller =

    do 

       putStrLn "Hit the Play button ..."

       loop controller

    where loop pidController  =

              do 
                  -- Parse the doubles into timed, vehicle state, and stick demands
                  let t = 0
                  let s = VehicleState 0 0 0 0 0 0 0 0 0 0 0 0

                  -- Get the function part of the PID controller
                  let controllerFun = fst pidController

                  -- Run the PID controller to get new demands
                  let (d, newControllerState) = controllerFun t s d (snd pidController)

                  loop (controllerFun, newControllerState)
