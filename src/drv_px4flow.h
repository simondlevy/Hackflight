#pragma once

// https://pixhawk.org/modules/px4flow#i2c
typedef struct px4flow_frame_t
{
    uint16_t frame_count;     // counts created I2C frames [#frames]
    int16_t pixel_flow_x_sum; // latest x flow measurement in pixels*10 [pixels]
    int16_t pixel_flow_y_sum; // latest y flow measurement in pixels*10 [pixels]
    int16_t flow_comp_m_x;    // x velocity*1000 [meters/sec]
    int16_t flow_comp_m_y;    // y velocity*1000 [meters/sec]
    int16_t qual;             // Optical flow quality / confidence [0: bad, 255: maximum quality]
    int16_t gyro_x_rate;      // latest gyro x rate [rad/sec]
    int16_t gyro_y_rate;      // latest gyro y rate [rad/sec]
    int16_t gyro_z_rate;      // latest gyro z rate [rad/sec]
    uint8_t gyro_range;       // gyro range [0 .. 7] equals [50 deg/sec .. 2000 deg/sec] 
    uint8_t sonar_timestamp;  // time since last sonar update [milliseconds]
    int16_t ground_distance;  // Ground distance in meters*1000 [meters]. +: known. -: unknown

} px4flow_frame_t;

uint8_t initPX4Flow();

void pollPX4Flow();
