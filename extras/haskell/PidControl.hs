{--
  PID control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PidControl(PidController,
                  pidFun,
                  pidState,
                  newPidController,
                  rateController,
                  yawController,
                  levelController,
                  altHoldController,
                  posHoldController) where
import State
import Demands

data FullPidState = 

    FullPidState { fullErrorIntegral :: Double,
                   fullDeltaError1 :: Double,
                   fullDeltaError2 :: Double,
                   fullErrorPrev :: Double }

data PidState =

     AltHoldState { altErrorIntegral :: Double,
                    altTarget :: Double,
                    altInBand :: Bool }

   | RateState { rateRollState :: FullPidState,
                 ratePitchState :: FullPidState }

   | YawState { yawErrorIntegral :: Double }

   | NoState { }

type PidFun = VehicleState -> Demands -> PidState -> (Demands, PidState)

data PidController = PidController { pidFun :: PidFun, pidState :: PidState }

newPidController :: PidFun -> PidState -> PidController
newPidController f s = PidController f s

-------------------------------------- Rate ----------------------------------

rateController :: Double -> Double -> Double -> Double -> Double ->
                  PidController

rateController kp ki kd windupMax rateMax = 
    PidController (rateClosure kp ki kd windupMax rateMax)
                  (RateState (FullPidState 0 0 0 0) (FullPidState 0 0 0 0))

rateClosure :: Double -> Double -> Double -> Double -> Double -> PidFun
rateClosure kp ki kd windupMax rateMax =

    \vehicleState -> \demands -> \controllerState ->

    let 

        computeDof demand angularVelocity oldPidState =

            let 

                --  Reset PID state on quick angular velocity change
                newPidState = if abs(angularVelocity) > deg2rad(rateMax)
                              then (FullPidState 0 0 0 0)
                              else oldPidState

                err = demand - angularVelocity

                errI = constrain_abs ((fullErrorIntegral newPidState) + err)
                                      windupMax
                deltaErr = err - (fullErrorPrev newPidState)
                errD = ((fullDeltaError1 newPidState) +
                        (fullDeltaError2 newPidState) +
                        deltaErr)

                in ((kp * err) + (ki * errI) + (kd * errD), 
                    (FullPidState (fullErrorIntegral newPidState)
                                  deltaErr  
                                  (fullDeltaError1 newPidState) 
                                  err))

        (rollDemand, rollPidState) = computeDof (Demands.roll demands)
                                                 (State.dphi vehicleState)
                                                 (rateRollState
                                                  controllerState)

        -- Pitch demand is nose-down positive, so we negate pitch-forward
        -- (nose-down negative) to reconcile them
        (pitchDemand, pitchPidState) = computeDof (Demands.pitch demands)
                                                  (-(State.dtheta vehicleState))
                                                  (ratePitchState
                                                   controllerState)

    -- Return updated demands and controller state
    in ((Demands (Demands.throttle demands)
                 rollDemand
                 pitchDemand
                 (Demands.yaw demands)),
        (RateState rollPidState pitchPidState))

----------------------------------- Yaw ---------------------------------------

yawController :: Double -> Double -> Double -> PidController

yawController kp ki windupMax = 
    PidController (yawClosure kp ki windupMax) (YawState 0)

yawClosure :: Double -> Double -> Double -> PidFun
yawClosure kp ki windupMax =

    \vehicleState -> \demands -> \controllerState ->

    -- Compute error as target minus actual
    let err = (Demands.yaw demands) - (State.dpsi vehicleState)

        -- Accumualte error integral
        errI = constrain_abs ((yawErrorIntegral controllerState) + err)
                             windupMax

    -- Return updated demands and controller state
    in (Demands (Demands.throttle demands) 
                (Demands.roll demands)
                (Demands.pitch demands)
                (kp * err + ki * errI),
        YawState errI)

---------------------------------- Level --------------------------------------

levelController :: Double -> Double -> PidController

levelController kp maxAngleDegrees =
    PidController (levelClosure kp maxAngleDegrees) NoState

levelClosure :: Double -> Double -> PidFun
levelClosure kp maxAngleDegrees =

    \vehicleState -> \demands -> \_controllerState ->

    let 
        -- Maximum roll pitch demand is +/-0.5, so to convert demand to
        -- angle for error computation, we multiply by the folling amount:
        demandScale = 2 * deg2rad(maxAngleDegrees)

    -- Return updated demands and controller state
    in (Demands (Demands.throttle demands) 

                -- New roll demand
                (kp * (Demands.roll demands) * demandScale -
                      (State.phi vehicleState))

                -- Pitch demand is nose-down positive, so we negate
                -- pitch-forward (nose-down negative) to reconcile them
                -- and get new pitch demand
                (kp * (Demands.pitch demands) * demandScale +
                      (State.theta vehicleState))

                (Demands.yaw demands),

        NoState)

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
         altitude = -(State.z vehicleState)

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
         err = altTargetVelocity + (State.dz vehicleState)

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

                        let p = State.psi vehicleState

                            -- Rotate X, Y velocities into body frame

                            cp = cos p
                            sp = sin p

                            xx = State.dx vehicleState
                            yy = State.dy vehicleState

                        in Demands (Demands.throttle demands)
                                   (-kp * (cp * xx + sp * yy))
                                   (-kp * (cp * yy - sp * xx))
                                   (Demands.yaw demands)

                    else demands

    in (newDemands, NoState)

--------------------------------- Helpers --------------------------------------

constrain_abs :: Double -> Double -> Double
constrain_abs v lim = if v < -lim then -lim else (if v > lim then lim else v)

in_band :: Double -> Double -> Bool
in_band value band = abs(value) < band

deg2rad :: Double -> Double
deg2rad d = d * pi / 180
