/**
 *
 * Copyright (C) 2011-2023 Bitcraze AB, 2024 Simon D. Levy
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
 *
 * Outlier rejection filter for the kalman filter
 */

#pragma once

#include <math.h>

#include "datatypes.h"


// This filter uses an "integrator" to track the ratio of good/samples. Samples
// with an errors (measurement - predicted) larger than the acceptance level
// will be discarded. The acceptance level is based on the integrator, with a
// hysteresis to avoid rapid changes. When the filter is open, all samples are
// let through, usually at startup to let the kalman filter converge. When the
// filter is closed, only samples with an error < the acceptance level, this
// should be most of the time.  The acceptance level is based on the standard
// deviation used for the tdoa samples, which should be based on the noise
// level in the system.

class OutlierFilterTdoa {

    public:

        void reset(void)
        {
            _integrator = 0.0f;
            _isFilterOpen = true;
            _latestUpdateMs = 0;
        }

        bool validateIntegrator(const tdoaMeasurement_t* tdoa, 
                const float error, const uint32_t nowMs)
        {
            // The accepted error when the filter is closed
            const float acceptedDistance = tdoa->stdDev * 2.5f;

            // The level used to determine if a sample is added or removed from
            // the integrator
            const float integratorTriggerDistance = tdoa->stdDev * 2.0f;


            bool sampleIsGood = false;

            // Discard samples that are physically impossible, most likely
            // measurement error
            if (isDistanceDiffSmallerThanDistanceBetweenAnchors(tdoa)) {
                uint32_t dtMs = nowMs - _latestUpdateMs;

                // Limit dt to minimize the impact on the integrator if we have
                // not received samples for a long time (or at start up)
                dtMs = fminf(dtMs, INTEGRATOR_SIZE / 10.0f);

                if (fabsf(error) < integratorTriggerDistance) {
                    _integrator += dtMs;
                    _integrator = fminf(_integrator, INTEGRATOR_SIZE);
                } else {
                    _integrator -= dtMs;
                    _integrator = fmaxf(_integrator, 0.0f);
                }

                if (_isFilterOpen) {
                    // The filter is open, let all samples through
                    sampleIsGood = true;

                    if (_integrator > INTEGRATOR_RESUME_ACTION_LEVEL) {
                        // We have recovered and converged, close the filter again
                        _isFilterOpen = false;
                    }
                } else {

                    // The filter is closed, let samples with a small error through
                    sampleIsGood = (fabsf(error) < acceptedDistance);

                    if (_integrator < INTEGRATOR_FORCE_OPEN_LEVEL) {

                        // We have got lots of outliers lately, the kalman
                        // filter may have diverged. Open up to try to recover
                        _isFilterOpen = true;
                    }
                }

                _latestUpdateMs = nowMs;
            }

            return sampleIsGood;
        }

    private:

        // The maximum size of the integrator. This size determines the time
        // [in ms] needed to open/close the filter.
        static constexpr float INTEGRATOR_SIZE = 300.0f;

        // The level when the filter open up to let all samples through
        static constexpr float INTEGRATOR_FORCE_OPEN_LEVEL = INTEGRATOR_SIZE * 0.1f;

        // The level when the filter closes again
        static constexpr float INTEGRATOR_RESUME_ACTION_LEVEL = INTEGRATOR_SIZE * 0.9f;

        static float square(float a) 
        {
            return a * a;
        }

        static float distanceSq(const point_t* a, const point_t* b) 
        {
            return square(a->x - b->x) + square(a->y - b->y) + square(a->z - b->z);
        }

        static bool isDistanceDiffSmallerThanDistanceBetweenAnchors(
                const tdoaMeasurement_t* tdoa) 
        {
            float anchorDistanceSq = distanceSq(
                    &tdoa->anchorPositions[0], &tdoa->anchorPositions[1]);
            float distanceDiffSq = square(tdoa->distanceDiff);
            return (distanceDiffSq < anchorDistanceSq);
        }

        float _integrator;
        uint32_t _latestUpdateMs;
        bool _isFilterOpen;

};

