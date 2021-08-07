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
import PidControllers
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

        -- Repclicate the receiver demands for mapping
        demandsList = [rxDemands|_<-[1..(length pidControllers)]]

        -- Map the PID update function to the pid controllers and demands
        control = map (pidUpdate vehicleState) (zip demandsList pidControllers)

        -- Extract the updated ew PID controllers
        pidControllers' = map (\p -> snd p) control

        -- Extract the updated list of demands
        demandsList' = map (\p -> fst p) control

        -- Sum over the list of demands to get the final demands
        demands = foldr addDemands (Demands 0 0 0 0) demandsList'

    -- Send the final demands to the mixer, returning the resulting motor
    -- values and the new PID controller states
    in ((getMotors mixer demands), pidControllers')
