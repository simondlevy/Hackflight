"""
   Mixer subclass for X-configuration quadcopters following the
   ArduPilot numbering convention:
    3cw   1ccw
       | /
        ^
       / |
    2ccw  4cw

Copyright (C) 2021 S.Basnet, N. Manaye, N. Nguyen, S.D. Levy
MIT License
"""

from demands import DEMANDS_THROTTLE, DEMANDS_PITCH, DEMANDS_ROLL, DEMANDS_YAW
import numpy as np


class Mixer(object):
    MAXMOTORS = 20

    def __init__(self, motordirs):
        self.motordirs = motordirs
        return

    def constrainMotorValue(index, value):
        return

    def run(self, demands):
        """
        Turn demands into motor spins in [0,1]
        """
        # return 0.6 * np.ones(4)

        # Map throttle demand from [-1,1] to [0,1]
        demands[DEMANDS_THROTTLE] = (demands[DEMANDS_THROTTLE]+1)/2

        motorvals = np.zeros(4)

        for i in range(4):  # motorvals[i]

            motorvals[i] = (demands[DEMANDS_THROTTLE] *
                            self.motordirs[i].throttle +

                            demands[DEMANDS_ROLL] *
                            self.motordirs[i].roll +

                            demands[DEMANDS_PITCH] *
                            self.motordirs[i].pitch +

                            demands[DEMANDS_YAW] *
                            self.motordirs[i].yaw)

        maxMotor = motorvals[0]

        for i in range(1, 4):
            if (motorvals[i] > maxMotor):
                maxMotor = motorvals[i]

        for i in range(4):
            # This is a way to still have good gyro corrections
            # if at least one motor reaches its max
            if (maxMotor > 1):
                motorvals[i] -= maxMotor - 1
            # Keep motor values in appropriate interval
            motorvals[i] = self.constrainMinMax(motorvals[i], 0, 1)

        return motorvals

    def constrainMinMax(self, val, min, max):
        if val < min:
            return min
        elif val > max:
            return max


# Replicating the motormixer struct from the C++ program
class motorMixer(object):

    def __init__(self, throttle=0, roll=0, pitch=0, yaw=0):

        self.throttle = throttle
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


"""
            // Custom mixer data per motor
            typedef struct motorMixer_t {
                int8_t throttle; // T
                int8_t roll; 	 // A
                int8_t pitch;	 // E
                int8_t yaw;	     // R
            } motorMixer_t;

  
            void run(float * demands)
            {
                // Map throttle demand from [-1,+1] to [0,1]
                demands[DEMANDS_THROTTLE] = (demands[DEMANDS_THROTTLE] + 1) / 2;

                float motorvals[MAXMOTORS];

                for (uint8_t i = 0; i < _nmotors; i++) {

                    motorvals[i] = 
                        (demands[DEMANDS_THROTTLE] * motorDirections[i].throttle + 
                         demands[DEMANDS_ROLL]     * motorDirections[i].roll +     
                         demands[DEMANDS_PITCH]    * motorDirections[i].pitch +   
                         demands[DEMANDS_YAW]      * motorDirections[i].yaw);      
                }

                float maxMotor = motorvals[0];

                for (uint8_t i = 1; i < _nmotors; i++)
                    if (motorvals[i] > maxMotor)
                        maxMotor = motorvals[i];

                for (uint8_t i = 0; i < _nmotors; i++) {

                    // This is a way to still have good gyro corrections if at least one motor reaches its max
                    if (maxMotor > 1) {
                        motorvals[i] -= maxMotor - 1;
                    }

                    // Keep motor values in appropriate interval
                    motorvals[i] = constrainMotorValue(i, motorvals[i]);
                }

                for (uint8_t i = 0; i < _nmotors; i++) {
                    safeWriteMotor(i, motorvals[i]);
                }
            }
"""
