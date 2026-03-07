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
                _bufferIndex = 0;
            }

            GyroBiasCalculator(
                    const Vec3 & biasOut,
                    const bool wasValueFound,
                    const SixAxisStats & stats,
                    const Vec3 & values,
                    const uint16_t bufferIndex,
                    const bool isBufferFilled,
                    const int32_t varianceSampleTime)
                : 
                biasOut(biasOut),
                wasValueFound(wasValueFound),
                _stats(stats),
                _values(values),
                _bufferIndex(bufferIndex),
                _varianceSampleTime(varianceSampleTime) { }

            GyroBiasCalculator& operator=(const GyroBiasCalculator& other)
                = default;

            static auto process(
                    const GyroBiasCalculator & calc,
                    const uint32_t ticks,
                    const axis3_i16_t gyroRaw) -> GyroBiasCalculator
            {
                _static_buffer[calc._bufferIndex].x = gyroRaw.x;
                _static_buffer[calc._bufferIndex].y = gyroRaw.y;
                _static_buffer[calc._bufferIndex].z = gyroRaw.z;

                const auto nextIndex = calc._bufferIndex + 1; 

                const auto isBufferFilled = (nextIndex == NBR_OF_SAMPLES);
 
                const auto bufferIndex = isBufferFilled ? 0 : nextIndex;

                const auto needUpdate = !calc.wasValueFound && isBufferFilled;

                const auto stats = needUpdate ? calculateStats(calc) : calc._stats;

                const auto valueWasFound = needUpdate &&
                    stats.variance.x < RAW_VARIANCE_BASE &&
                    stats.variance.y < RAW_VARIANCE_BASE &&
                    stats.variance.z < RAW_VARIANCE_BASE &&
                    calc._varianceSampleTime + MIN_BIAS_TIMEOUT_MS < ticks;

                const auto varianceSampleTime =
                    valueWasFound ? ticks : calc._varianceSampleTime;

                const auto values = valueWasFound ? stats.mean : calc._stats.mean;

                (void)varianceSampleTime;
                (void)values;
                (void)bufferIndex;

                return GyroBiasCalculator();
            }

            static void process(
                    GyroBiasCalculator & calc,
                    const uint32_t ticks,
                    const axis3_i16_t gyroRaw)
            {
                calc._buffer[calc._bufferIndex].x = gyroRaw.x;
                calc._buffer[calc._bufferIndex].y = gyroRaw.y;
                calc._buffer[calc._bufferIndex].z = gyroRaw.z;
                calc._bufferIndex++;

                auto isBufferFilled = calc._bufferIndex == NBR_OF_SAMPLES;

                if (isBufferFilled) {
                    calc._bufferIndex = 0;
                }

                if (!calc.wasValueFound && isBufferFilled) {

                    calc._stats = calculateStats(calc);

                    if (
                            calc._stats.variance.x < RAW_VARIANCE_BASE &&
                            calc._stats.variance.y < RAW_VARIANCE_BASE &&
                            calc._stats.variance.z < RAW_VARIANCE_BASE &&
                            (calc._varianceSampleTime + MIN_BIAS_TIMEOUT_MS < ticks))
                    {
                        calc._varianceSampleTime = ticks;
                        calc._values = calc._stats.mean;
                        calc.wasValueFound = true;
                    }
                }

                calc.biasOut = calc._values;
            }

        private:

            SixAxisStats _stats;
            Vec3 _values;
            axis3_i16_t _buffer[NBR_OF_SAMPLES];
            uint16_t _bufferIndex;
            bool _isBufferFilled;
            int32_t _varianceSampleTime;

            static axis3_i16_t _static_buffer[NBR_OF_SAMPLES];

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
