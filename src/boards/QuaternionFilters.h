/*  Header file for Kris Winer's quaternion-filter support
 *
 *  Copyright 2017 Tlera Corporation
 *  
 *  Created by Kris Winer
 *
 *  Adapted by Simon D. Levy 19 April 2018
 *  
 *  Library may be used freely and without limit with attribution.
 */

/** Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
* (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
* which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
* device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
* The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
* but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
*/
class MadgwickQuaternion {

    public:

        MadgwickQuaternion(float beta);

        __attribute__((optimize("O3")))  
        void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat, float q[4]);

    private:

        float _beta;
};

/** Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
 * measured ones. 
 */
class MahonyQuaternion {

    public:

        MahonyQuaternion(void);

        void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat, float q[4]);

    private:

        float _eInt[3];
};
