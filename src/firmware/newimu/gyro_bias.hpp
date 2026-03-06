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

    class GyroBiasCalculator {

        private:

            // Number of samples used in variance calculation. Changing this
            // affects the threshold
            static const uint16_t NBR_OF_SAMPLES = 512;

            static constexpr float RAW_VARIANCE_BASE = 100;
            static const uint32_t MIN_BIAS_TIMEOUT_MS = 1000;

        public:

            Vec3 biasOut;
            bool wasValueFound;

            GyroBiasCalculator() 
            {
                wasValueFound = false;
                _isBufferFilled = false;
                _bufHead = _buffer;
            }

            /*
            GyroBiasCalculator(
                    const Vec3 & biasOut,
                    const bool wasValueFound,
                    const SixAxisStats & stats,
                    const Vec3 & values,
                    const bool isBufferFilled,
                    axis3_i16_t * bufHead,
                    const int32_t varianceSampleTime) 
                :
                    biasOut(biasOut),
                    wasValueFound(wasValueFound),
                    _stats(stats),
                    _values(values),
                    _isBufferFilled(isBufferFilled),
                    _bufHead(bufHead),
                    _varianceSampleTime(varianceSampleTime) {}*/

            GyroBiasCalculator& operator=(const GyroBiasCalculator& other)
                = default;

            static void process(
                    GyroBiasCalculator & calc,
                    const uint32_t ticks,
                    const axis3_i16_t gyroRaw)
            {
                calc._bufHead->x = gyroRaw.x;
                calc._bufHead->y = gyroRaw.y;
                calc._bufHead->z = gyroRaw.z;
                calc._bufHead++;

                if (calc._bufHead >= &calc._buffer[NBR_OF_SAMPLES]) {

                    calc._bufHead = calc._buffer;
                    calc._isBufferFilled = true;
                }

                if (!calc.wasValueFound && calc._isBufferFilled) {

                    calc._stats = calculateStats(calc);

                    if (
                            calc._stats.variance.x < RAW_VARIANCE_BASE &&
                            calc._stats.variance.y < RAW_VARIANCE_BASE &&
                            calc._stats.variance.z < RAW_VARIANCE_BASE &&
                            (calc._varianceSampleTime + MIN_BIAS_TIMEOUT_MS < ticks))
                    {
                        calc._varianceSampleTime = ticks;
                        calc._values.x = calc._stats.mean.x;
                        calc._values.y = calc._stats.mean.y;
                        calc._values.z = calc._stats.mean.z;
                        calc.wasValueFound = true;
                    }
                }

                calc.biasOut.x = calc._values.x;
                calc.biasOut.y = calc._values.y;
                calc.biasOut.z = calc._values.z;
            }

        private:

            SixAxisStats _stats;
            Vec3 _values;
            bool _isBufferFilled;
            axis3_i16_t * _bufHead;
            axis3_i16_t _buffer[NBR_OF_SAMPLES];
            int32_t _varianceSampleTime;

            static auto calculateStats(const GyroBiasCalculator & calc)
                -> SixAxisStats
                {
                    int64_t xsum=0, ysum=0, zsum=0;
                    int64_t xsumsq=0, ysumsq=0, zsumsq=0;

                    for (uint16_t i=0; i<NBR_OF_SAMPLES; i++) {

                        xsum += calc._buffer[i].x;
                        ysum += calc._buffer[i].y;
                        zsum += calc._buffer[i].z;
                        xsumsq += calc._buffer[i].x * calc._buffer[i].x;
                        ysumsq += calc._buffer[i].y * calc._buffer[i].y;
                        zsumsq += calc._buffer[i].z * calc._buffer[i].z;
                    }

                    const auto mean = Vec3(
                            (float) xsum / NBR_OF_SAMPLES,
                            (float) ysum / NBR_OF_SAMPLES,
                            (float) zsum / NBR_OF_SAMPLES);

                    const auto variance = Vec3(
                            xsumsq / NBR_OF_SAMPLES - mean.x * mean.x,
                            ysumsq / NBR_OF_SAMPLES - mean.y * mean.y,
                            zsumsq / NBR_OF_SAMPLES - mean.z * mean.z);

                    return SixAxisStats(mean, variance);
                }

    }; // class GyroBias

} // namespace hf
