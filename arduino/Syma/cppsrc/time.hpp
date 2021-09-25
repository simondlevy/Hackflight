/*
   Timer task support

   Copyright (c) 2018 Simon D. Levy

   MIT License
   */

#pragma once

namespace hf {

    static bool ready(const uint32_t freq)
    {
        const uint32_t period = 1000000 / freq;
        static uint32_t micros_prev;
        uint32_t micros = copilot_micros;
        micros_prev = (micros - micros_prev) > period ? micros : micros_prev;
    }

} // namespace hf
