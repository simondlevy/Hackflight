/* Copyright (C) 2011-2018 Bitcraze AB, 2025 Simon D. Levy * * This program
 * is free software: you can redistribute it and/or modify * it under the terms
 * of the GNU General Public License as published by * the Free Software
 * Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <datatypes.hpp>
#include <firmware/newimu/six_axis_stats.hpp>

namespace hf {

    class GyroBias {

        private:

            // Number of samples used in variance calculation. Changing this
            // affects the threshold
            static const uint16_t NBR_OF_SAMPLES = 512;

            static constexpr float RAW_VARIANCE_BASE = 100;
            static const uint32_t MIN_BIAS_TIMEOUT_MS = 1000;

        public:

            GyroBias() 
            {
                _isBufferFilled = false;
                _bufHead = _buffer;
            }

            /**
             * Checks if the variances is below the predefined thresholds.
             */
            /**
             * Calculates the bias first when the gyro variance is below threshold.
             * Requires a buffer but calibrates platform first when it is stable.
             */
            static bool process(
                    GyroBias & bias,
                    const uint32_t tickCount,
                    const axis3_i16_t gyroRaw,
                    Vec3 & gyroBiasOut)
            {
                bias._bufHead->x = gyroRaw.x;
                bias._bufHead->y = gyroRaw.y;
                bias._bufHead->z = gyroRaw.z;
                bias._bufHead++;

                if (bias._bufHead >= 
                        &bias._buffer[NBR_OF_SAMPLES]) {

                    bias._bufHead = bias._buffer;
                    bias._isBufferFilled = true;
                }

                if (!bias._wasValueFound) {
                    GyroBias::findValue(bias, tickCount);
                }

                gyroBiasOut.x = bias._values.x;
                gyroBiasOut.y = bias._values.y;
                gyroBiasOut.z = bias._values.z;

                return bias._wasValueFound;
            }

        private:

            SixAxisStats _stats;
            Vec3 _values;

            bool _wasValueFound;
            bool _isBufferFilled;
            axis3_i16_t * _bufHead;
            axis3_i16_t _buffer[NBR_OF_SAMPLES];

            static void calculateStats(const GyroBias & bias, SixAxisStats & stats)
            {
                int64_t sum[3] = {};
                int64_t sumSq[3] = {};

                for (uint16_t i=0; i<NBR_OF_SAMPLES; i++) {

                    sum[0] += bias._buffer[i].x;
                    sum[1] += bias._buffer[i].y;
                    sum[2] += bias._buffer[i].z;
                    sumSq[0] += bias._buffer[i].x * bias._buffer[i].x;
                    sumSq[1] += bias._buffer[i].y * bias._buffer[i].y;
                    sumSq[2] += bias._buffer[i].z * bias._buffer[i].z;
                }

                stats.mean.x = (float) sum[0] / NBR_OF_SAMPLES;
                stats.mean.y = (float) sum[1] / NBR_OF_SAMPLES;
                stats.mean.z = (float) sum[2] / NBR_OF_SAMPLES;

                stats.variance.x =
                    sumSq[0] / NBR_OF_SAMPLES - stats.mean.x * stats.mean.x;
                stats.variance.y =
                    sumSq[1] / NBR_OF_SAMPLES - stats.mean.y * stats.mean.y;
                stats.variance.z =
                    sumSq[2] / NBR_OF_SAMPLES - stats.mean.z * stats.mean.z;
            }

            static void findValue(GyroBias & bias, const uint32_t ticks)
            {
                static int32_t varianceSampleTime;

                if (bias._isBufferFilled)
                {
                    calculateStats(bias, bias._stats);

                    if (
                            bias._stats.variance.x < RAW_VARIANCE_BASE &&
                            bias._stats.variance.y < RAW_VARIANCE_BASE &&
                            bias._stats.variance.z < RAW_VARIANCE_BASE &&
                            (varianceSampleTime + MIN_BIAS_TIMEOUT_MS < ticks))
                    {
                        varianceSampleTime = ticks;
                        bias._values.x = bias._stats.mean.x;
                        bias._values.y = bias._stats.mean.y;
                        bias._values.z = bias._stats.mean.z;
                        bias._wasValueFound = true;
                    }
                }
            }

    }; // class GyroBias

} // namespace hf
