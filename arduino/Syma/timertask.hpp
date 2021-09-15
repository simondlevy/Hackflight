/*
   Abstract class for timer tasks

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "debugger.hpp"
#include "state.hpp"

#include "copilot_extra.h"

namespace hf {

    class TimerTask {

        private:

            float _period = 0;
            float _time = 0;

        protected:

            TimerTask(float freq)
            {
                _period = 1 / freq;
                _time = 0;
            }

            bool ready(void)
            {
                float time = copilot_time_sec;

                if ((time - _time) > _period)
                {
                    _time = time;
                    return true;
                }

                return false;
             }

    };  // TimerTask

} // namespace hf
