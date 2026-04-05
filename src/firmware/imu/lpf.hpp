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

namespace hf {

    class LPF {

        public:

            float output;

            LPF() : output(0), _delay1(0), _delay2(0) {}

            LPF(const float output, const float delay1, const float delay2)
                : output(output), _delay1(0), _delay2(0) {}

            LPF& operator=(const LPF& other) = default;

            static auto apply(
                    const LPF & lpf,
                    const float sample,
                    const float cutoff_freq,
                    const float sample_freq=1000) -> LPF
            {
                const auto fr = sample_freq/cutoff_freq;
                const auto ohm = tanf(M_PI/fr);
                const auto c = 1+2*cosf(M_PI/4)*ohm+ohm*ohm;

                const auto b0 = ohm*ohm/c;
                const auto b1 = 2*b0;
                const auto a1 = 2*(ohm*ohm-1)/c;
                const auto a2 = (1-2*cosf(M_PI/4)*ohm+ohm*ohm)/c;

                const auto try_delay0 = sample - lpf._delay1 * a1 - lpf._delay2 * a2;

                // don't allow bad values to propigate through the filter
                const auto delay0 = isfinite(try_delay0) ? try_delay0 : sample;

                const auto output = delay0 * b0 + lpf._delay1 * b1 + lpf._delay2 * b0;

                return LPF(delay0, lpf._delay1, output);
            }

        private:

            float _delay1;
            float _delay2;

    }; // class Lpf

} // namespace hf
