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

#include <firmware/datatypes.hpp>

namespace hf {

    class GyroBiasCalculator {

        private:

            static constexpr float RAW_VARIANCE_BASE = 100;
            static const uint32_t MIN_BIAS_TIMEOUT_MS = 1000;

        public:

            // Number of samples used in variance calculation. Changing this
            // affects the threshold
            static const uint16_t NBR_OF_SAMPLES = 512;

            Vec3 biasOut;
            bool wasValueFound;
            uint16_t bufferIndex;

            GyroBiasCalculator() = default;

            GyroBiasCalculator& operator=(const GyroBiasCalculator& other)
                = default;

            GyroBiasCalculator(
                    const Vec3 & biasOut,
                    const bool wasValueFound,
                    const uint16_t bufferIndex,
                    const ThreeAxisStats & stats,
                    const Vec3 & values,
                    const bool isBufferFilled,
                    const int32_t varianceSampleTime)
                :                     
                    biasOut(biasOut),
                    wasValueFound(wasValueFound),
                    bufferIndex(bufferIndex),
                    _stats(stats),
                    _values(values),
                    _isBufferFilled(isBufferFilled),
                    _varianceSampleTime(varianceSampleTime) {}

            static auto process(
                    const GyroBiasCalculator & calc,
                    const Vec3Raw * buffer,
                    const uint32_t ticks) -> GyroBiasCalculator
            {
                const auto bufferIndex = calc.bufferIndex + 1;

                const auto isBufferFilled = bufferIndex == NBR_OF_SAMPLES;

                const auto wantUpdate = !calc.wasValueFound && isBufferFilled;

                const auto stats = wantUpdate ? calculateStats(buffer) : calc._stats;

                const auto shouldUpdate = wantUpdate &&
                    stats.variance.x < RAW_VARIANCE_BASE &&
                    stats.variance.y < RAW_VARIANCE_BASE &&
                    stats.variance.z < RAW_VARIANCE_BASE &&
                    calc._varianceSampleTime + MIN_BIAS_TIMEOUT_MS < ticks;

                const auto values = shouldUpdate ? stats.mean : calc._values;

                const auto varianceSampleTime = shouldUpdate ? ticks : calc._varianceSampleTime;

                const auto wasValueFound = shouldUpdate ? true : calc.wasValueFound;


                const auto newBufferIndex = isBufferFilled ? 0 : bufferIndex;

                return GyroBiasCalculator(
                        values,
                        wasValueFound,
                        newBufferIndex,
                        stats,
                        values,
                        isBufferFilled,
                        varianceSampleTime);
            }

        private:

            ThreeAxisStats _stats;
            Vec3 _values;
            bool _isBufferFilled;
            int32_t _varianceSampleTime;

            static auto calculateStats(const Vec3Raw * buffer)
                -> ThreeAxisStats
                {
                    int64_t xsum=0, ysum=0, zsum=0;
                    int64_t xsumsq=0, ysumsq=0, zsumsq=0;

                    for (uint16_t i=0; i<NBR_OF_SAMPLES; i++) {

                        printf("values,%d,%d,%d\n", 
                                buffer[i].x, buffer[i].y, buffer[i].z);

                        xsum += buffer[i].x;
                        ysum += buffer[i].y;
                        zsum += buffer[i].z;
                        xsumsq += buffer[i].x * buffer[i].x;
                        ysumsq += buffer[i].y * buffer[i].y;
                        zsumsq += buffer[i].z * buffer[i].z;
                    }

                    const auto mean = Vec3(
                            (float) xsum / NBR_OF_SAMPLES,
                            (float) ysum / NBR_OF_SAMPLES,
                            (float) zsum / NBR_OF_SAMPLES);

                    const auto variance = Vec3(
                            (float)xsumsq / NBR_OF_SAMPLES - mean.x * mean.x,
                            (float)ysumsq / NBR_OF_SAMPLES - mean.y * mean.y,
                            (float)zsumsq / NBR_OF_SAMPLES - mean.z * mean.z);

                    printf("\noldmean,%f,%f,%f\n",
                            mean.x, mean.y, mean.z);
                    printf("oldvariance,%f,%f,%f\n\n",
                            variance.x, variance.y, variance.z);

                    return ThreeAxisStats(mean, variance);
                }

    }; // class GyroBias

} // namespace hf
