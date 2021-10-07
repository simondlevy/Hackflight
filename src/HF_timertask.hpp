/*
   Abstract class for timer tasks

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

namespace hf {

    class TimerTask {

        private:

            uint32_t _period = 0;
            uint32_t _time_prev = 0;

        protected:

            TimerTask(uint32_t freq)
            {
                _period = 1000000 / freq;
                _time_prev = 0;
            }

            bool ready(uint32_t time_usec)
            {
                if ((time_usec - _time_prev) > _period)
                {
                    _time_prev = time_usec;
                    return true;
                }

                return false;
             }

    };  // TimerTask

} // namespace hf
