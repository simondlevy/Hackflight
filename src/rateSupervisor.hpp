/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
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

#include <stdint.h>

class RateSupervisor {

    private:

        uint32_t _count;
        uint32_t _expectedMin;
        uint32_t _expectedMax;
        uint32_t _nextEvaluationTimeMs;
        uint32_t _evaluationIntervalMs;
        uint32_t _latestCount;
        uint8_t _skip;

    public:

        /**
         * @brief Initialize a rateSupervisor_t struct for rate measurements
         *
         * @param osTimeMs The current os time in ms
         * @param evaluationIntervalMs How often to evaluate the rate, in ms
         * @param minCount The minimum number of validations we expect every evaluation interval
         * @param maxCount The maximum number of validations we expect every evaluation interval
         * @param skip The number of inital evaluations to ignore failures for. This is a convenient way to let the system warm up a bit before reporting problems.
         */
        void init(const uint32_t osTimeMs, const uint32_t evaluationIntervalMs, const uint32_t minCount, const uint32_t maxCount, const uint8_t skip)
        {
            _count = 0;
            _evaluationIntervalMs = evaluationIntervalMs;
            _expectedMin = minCount;
            _expectedMax = maxCount;
            _nextEvaluationTimeMs = osTimeMs + evaluationIntervalMs;
            _latestCount = 0;
            _skip = skip;
        }

        /**
         * @brief Validate the rate for a process. This function should be called from the process for which the rate
         * is to be supervised. When the function is called a counter is increased, and if the evaluation period
         * has passed, the rate is evaluated.
         *
         * @param osTimeMs The current os time in ms
         * @return true if the measured rate is within bounds, or we have not yet reached the next evaluation time
         * @return false if the measured rate is too low or high
         */
        bool validate(const uint32_t osTimeMs)
        {
            bool result = true;

            _count += 1;
            if (osTimeMs > _nextEvaluationTimeMs) {
                uint32_t actual = _count;
                if (actual < _expectedMin || actual > _expectedMax) {
                    result = false;
                }

                _latestCount = _count;
                _count = 0;
                _nextEvaluationTimeMs = osTimeMs + _evaluationIntervalMs;

                if (_skip > 0) {
                    result = true;
                    _skip -= 1;
                }
            }

            return result;
        }

        /**
         * @brief Get the latest count. Useful to display the count after a failed validation.
         *
         * @return uint32_t The count at the latest evaluation time
         */
        uint32_t getLatestCount(void)
        {
            return _latestCount;
        }

}; // class RateSupervisor
