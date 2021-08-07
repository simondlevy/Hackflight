{--
  Hackflight core algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight(HackflightFun, hackflight)
where

import Demands
import Receiver(Receiver, getDemands)
import Sensor(Sensor, modifyState)
import Mixer(Mixer, getMotors)
import Motor(Motors)
import PidControl
import VehicleState(initialVehicleState)

type HackflightFun = Receiver ->
                     [Sensor] ->
                     [PidController] ->
                     Mixer ->
                     (Motors, [PidController])

hackflight :: HackflightFun

hackflight receiver sensors pidControllers mixer =

    let rxDemands = getDemands receiver

        -- Get the vehicle state by running the sensors
        vehicleState = foldr modifyState initialVehicleState sensors

        -- Repclicate receiver demands for mapping
        dlist = [rxDemands|_<-[1..(length pidControllers)]]

        -- Map the PID update function to the pid controllers and demands
        newControl = map (pidUpdate vehicleState) (zip dlist pidControllers)

        -- Extract the new PID controllers
        newPidControllers = map (\p -> snd p) newControl

        -- Extract the new list of demands
        newDemandsList = map (\p -> fst p) newControl

        -- Sum over the list of demands to get the final demands
        newDemands = foldr addDemands (Demands 0 0 0 0) newDemandsList

    -- Send the final demands to the mixer, returning the resulting motor
    -- values and the new PID controller states
    in ((getMotors mixer newDemands), newPidControllers)
