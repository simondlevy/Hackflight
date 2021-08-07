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

        vehicleState = foldr modifyState initialVehicleState sensors

        -- Repclicate receiver demands for mapping
        dlist = [rxDemands|_<-[1..(length pidControllers)]]

        combo = zip dlist pidControllers

        foo = map (pidUpdate vehicleState) combo

        newNewPidControllers = map (\p -> snd p) foo

        newDemandsList = map (\p -> fst p) foo

        newDemands = foldr addDemands (Demands 0 0 0 0) newDemandsList

    in ((getMotors mixer newDemands), newNewPidControllers)

addDemands :: Demands -> Demands -> Demands
addDemands d1 d2 = Demands ((throttle d1) + (throttle d2))
                           ((roll d1) + (roll d2))
                           ((pitch d1) + (pitch d2))
                           ((yaw d1) + (yaw d2))
  

