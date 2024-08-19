/*
  Hackflight atittude-calibration sketch

  Adapted from https://github.com/nickrehm/dRehmFlight
 
  Copyright (C) 2024 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

static void calibrateAttitude() {
    //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to
    //converge before commands can be sent to the actuators Assuming vehicle is
    //powered up on level surface!
    /*
     * This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds
     * to boot. 
     */
    //Warm up IMU and madgwick filter in simulated main loop
    for (int i = 0; i <= 10000; i++) {
        usec_prev = usec_curr;      
        usec_curr = micros();      
        dt = (usec_curr - usec_prev)/1000000.0; 
        getIMUdata();
        Madgwick6DOF(dt, GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, roll_IMU, pitch_IMU, yaw_IMU);
        loopRate(2000); //do not exceed 2000Hz
    }
}


static void calculate_IMU_error() 
{
    //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note:
    //vehicle should be powered up on flat surface
    /*
     * Don't worry too much about what this is doing. The error values it
     * computes are applied to the raw gyro and accelerometer values AccX,
     * AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift
     * in the
     * measurement. 
     */
    int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
    ACC_ERROR_X = 0.0;
    ACC_ERROR_Y = 0.0;
    ACC_ERROR_Z = 0.0;
    GYRO_ERROR_X = 0.0;
    GYRO_ERROR_Y= 0.0;
    GYRO_ERROR_Z = 0.0;

    //Read IMU values 12000 times
    int c = 0;
    while (c < 12000) {

        mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

        AccX  = AcX / ACCEL_SCALE_FACTOR;
        AccY  = AcY / ACCEL_SCALE_FACTOR;
        AccZ  = AcZ / ACCEL_SCALE_FACTOR;
        GyroX = GyX / GYRO_SCALE_FACTOR;
        GyroY = GyY / GYRO_SCALE_FACTOR;
        GyroZ = GyZ / GYRO_SCALE_FACTOR;

        //Sum all readings
        ACC_ERROR_X  = ACC_ERROR_X + AccX;
        ACC_ERROR_Y  = ACC_ERROR_Y + AccY;
        ACC_ERROR_Z  = ACC_ERROR_Z + AccZ;
        GYRO_ERROR_X = GYRO_ERROR_X + GyroX;
        GYRO_ERROR_Y = GYRO_ERROR_Y + GyroY;
        GYRO_ERROR_Z = GYRO_ERROR_Z + GyroZ;
        c++;
    }
    //Divide the sum by 12000 to get the error value
    ACC_ERROR_X  = ACC_ERROR_X / c;
    ACC_ERROR_Y  = ACC_ERROR_Y / c;
    ACC_ERROR_Z  = ACC_ERROR_Z / c - 1.0;
    GYRO_ERROR_X = GYRO_ERROR_X / c;
    GYRO_ERROR_Y = GYRO_ERROR_Y / c;
    GYRO_ERROR_Z = GYRO_ERROR_Z / c;

    Serial.print("float ACC_ERROR_X = ");
    Serial.print(ACC_ERROR_X);
    Serial.println(";");
    Serial.print("float ACC_ERROR_Y = ");
    Serial.print(ACC_ERROR_Y);
    Serial.println(";");
    Serial.print("float ACC_ERROR_Z = ");
    Serial.print(ACC_ERROR_Z);
    Serial.println(";");

    Serial.print("float GYRO_ERROR_X = ");
    Serial.print(GYRO_ERROR_X);
    Serial.println(";");
    Serial.print("float GYRO_ERROR_Y = ");
    Serial.print(GYRO_ERROR_Y);
    Serial.println(";");
    Serial.print("float GYRO_ERROR_Z = ");
    Serial.print(GYRO_ERROR_Z);
    Serial.println(";");

    Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
}


void setup() 
{

}

void loop() 
{
}
