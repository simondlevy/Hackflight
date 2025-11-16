/**
 * Copyright (C) 2025 Simon D. Levy
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

#define _MAIN

#include <debugger.hpp>
#include <estimator.hpp>
#include <tasks/core.hpp>
#include <tasks/opticalflow.hpp>
#include <tasks/zranger.hpp>

class Hackflight {

    public:

        void init(const uint8_t motorCount, const mixFun_t mixFun)
        {
            zrangerTask.begin(&estimator);

            opticalFlowTask.begin(&estimator);

            estimator.begin();

            coreTask.begin(&estimator, motorCount, mixFun);
        }

    private:

        CoreTask coreTask;
        Estimator estimator;
        OpticalFlowTask opticalFlowTask;
        ZRangerTask zrangerTask;
        Debugger debugger;
};
