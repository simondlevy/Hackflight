{--
  PID control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PidControllers(PidController,
                  altHoldController,
                  posHoldController) where

import VehicleState
import PidControl
import Demands
import Utils(constrain_abs, in_band)

----------------------------- Altitude hold -----------------------------------

altHoldController :: Double -> Double -> Double -> Double -> Double 
                        -> PidController
altHoldController kp ki windupMax pilotVelZMax stickDeadband = 
    PidController (altHoldClosure kp ki windupMax pilotVelZMax stickDeadband)
                  (AltHoldState 0 0 False)

altHoldClosure :: Double -> Double -> Double -> Double -> Double -> PidFun
altHoldClosure kp ki windupMax pilotVelZMax stickDeadband =

    \vehicleState -> \demands -> \controllerState ->

    let  
         -- NED => ENU
         altitude = -(VehicleState.z vehicleState)

         throttleDemand = throttle demands

         inband = in_band throttleDemand stickDeadband

         -- Reset controller when moving into deadband
         newControllerState = if inband && not (altInBand controllerState)
                              then AltHoldState 0 altitude True
                              else controllerState

         -- Target velocity is a setpoint inside deadband, scaled
         -- constant outside
         altTargetVelocity = if inband
                          then (altTarget newControllerState) - altitude
                          else pilotVelZMax * throttleDemand


         -- Compute error as altTarget velocity minus actual velocity, after
         -- negating actual to accommodate NED
         err = altTargetVelocity + (VehicleState.dz vehicleState)

         -- Accumualte error integral
         errI = constrain_abs ((altErrorIntegral controllerState) + err)
                windupMax

    -- Return updated demands and controller state
    in  (Demands (err * kp + errI * ki)
                 (Demands.roll demands)
                 (Demands.pitch demands)
                 (Demands.yaw demands),
         AltHoldState errI (altTarget newControllerState) inband)
                      

---------------------------- Position hold ------------------------------------

posHoldController :: Double -> Double -> PidController

posHoldController kp stickDeadband =
    PidController (posHoldClosure kp stickDeadband) NoState

posHoldClosure :: Double -> Double -> PidFun
posHoldClosure kp stickDeadband =

    \vehicleState -> \demands -> \_controllerState ->

    let newDemands = if in_band (Demands.roll demands) stickDeadband &&
                        in_band (Demands.pitch demands) stickDeadband
        
                    then

                        let p = VehicleState.psi vehicleState

                            -- Rotate X, Y velocities into body frame

                            cp = cos p
                            sp = sin p

                            xx = VehicleState.dx vehicleState
                            yy = VehicleState.dy vehicleState

                        in Demands (Demands.throttle demands)
                                   (-kp * (cp * xx + sp * yy))
                                   (-kp * (cp * yy - sp * xx))
                                   (Demands.yaw demands)

                    else demands

    in (newDemands, NoState)
