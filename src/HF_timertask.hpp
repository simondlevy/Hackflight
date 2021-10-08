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
            uint32_t _time_usec;

        protected:

            TimerTask(uint32_t freq)
            {
                _period = 1000000 / freq;
                _time_usec = 0;
            }

            bool ready(uint32_t time_usec)
            {
                if ((time_usec - _time_usec) > _period)
                {
                    _time_usec = time_usec;
                    return true;
                }

                return false;
             }

    };  // TimerTask

} // namespace hf
