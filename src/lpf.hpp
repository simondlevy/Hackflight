/**
 * Copyright (C) 2011-2012 Bitcraze AB, 2024 Simon D. Levy
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
 * filter.h - Filtering functions
 */

#pragma once

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "physicalConstants.h"

class Lpf {

    private:

        float _a1;
        float _a2;
        float _b0;
        float _b1;
        float _b2;
        float _delay_element_1;
        float _delay_element_2;

        /** Second order low pass filter structure.
         *
         * using biquad filter with bilinear z transform
         *
         * http://en.wikipedia.org/wiki/Digital_biquad_filter
         * http://www.earlevel.com/main/2003/03/02/the-bilinear-z-transform
         *
         * Laplace continious form:
         *
         *                 1
         * H(s) = -------------------
         *        s^2/w^2 + s/w*Q + 1
         *
         *
         * Polynomial discrete form:
         *
         *        b0 + b1 z^-1 + b2 z^-2
         * H(z) = ----------------------
         *        a0 + a1 z^-1 + a2 z^-2
         *
         * with:
         *  a0 = 1
         *  a1 = 2*(K^2 - 1) / (K^2 + K/Q + 1)
         *  a2 = (K^2 - K/Q + 1) / (K^2 + K/Q + 1)
         *  b0 = K^2 / (K^2 + K/Q + 1)
         *  b1 = 2*b0
         *  b2 = b0
         *  K = tan(pi*Fc/Fs) ~ pi*Fc/Fs = Ts/(2*tau)
         *  Fc: cutting frequency
         *  Fs: sampling frequency
         *  Ts: sampling period
         *  tau: time constant (tau = 1/(2*pi*Fc))
         *  Q: gain at cutoff frequency
         *
         * Note that b[0]=b[2], so we don't need to save b[2]
         */
        typedef struct SecondOrderLowPass {
          float a[2]; ///< denominator gains
          float b[2]; ///< numerator gains
          float i[2]; ///< input history
          float o[2]; ///< output history
        } SecondOrderLowPass_t;

        typedef struct SecondOrderLowPass_int {
            int32_t a[2]; ///< denominator gains
            int32_t b[2]; ///< numerator gains
            int32_t i[2]; ///< input history
            int32_t o[2]; ///< output history
            int32_t loop_gain; ///< loop gain
        } SecondOrderLowPassInt_t;

        /** Second order Butterworth low pass filter.
         */
        typedef struct SecondOrderLowPass Butterworth2LowPass_t;

        /** Init second order low pass filter.
         *
         * @param filter second order low pass filter structure
         * @param tau time constant of the second order low pass filter
         * @param Q Q value of the second order low pass filter
         * @param sample_time sampling period of the signal
         * @param value initial value of the filter
         */
        static inline void init_second_order_low_pass(struct SecondOrderLowPass *filter, float tau, float Q, float sample_time,
                float value)
        {
            float K = tanf(sample_time / (2.0f * tau));
            float poly = K * K + K / Q + 1.0f;
            filter->a[0] = 2.0f * (K * K - 1.0f) / poly;
            filter->a[1] = (K * K - K / Q + 1.0f) / poly;
            filter->b[0] = K * K / poly;
            filter->b[1] = 2.0f * filter->b[0];
            filter->i[0] = filter->i[1] = filter->o[0] = filter->o[1] = value;
        }

        /** Update second order low pass filter state with a new value.
         *
         * @param filter second order low pass filter structure
         * @param value new input value of the filter
         * @return new filtered value
         */
        static inline float update_second_order_low_pass(struct SecondOrderLowPass *filter, float value)
        {
            float out = filter->b[0] * value
                + filter->b[1] * filter->i[0]
                + filter->b[0] * filter->i[1]
                - filter->a[0] * filter->o[0]
                - filter->a[1] * filter->o[1];
            filter->i[1] = filter->i[0];
            filter->i[0] = value;
            filter->o[1] = filter->o[0];
            filter->o[0] = out;
            return out;
        }

        /** Get current value of the second order low pass filter.
         *
         * @param filter second order low pass filter structure
         * @return current value of the filter
         */
        static inline float get_second_order_low_pass(struct SecondOrderLowPass *filter)
        {
            return filter->o[0];
        }


        /** Init a second order Butterworth filter.
         *
         * based on the generic second order filter
         * with Q = 0.7071 = 1/sqrt(2)
         *
         * http://en.wikipedia.org/wiki/Butterworth_filter
         *
         * @param filter second order Butterworth low pass filter structure
         * @param tau time constant of the second order low pass filter
         * @param sample_time sampling period of the signal
         * @param value initial value of the filter
         */
        static inline void init_butterworth_2_low_pass(Butterworth2LowPass_t *filter, float tau, float sample_time, float value)
        {
            init_second_order_low_pass((struct SecondOrderLowPass *)filter, tau, 0.7071, sample_time, value);
        }

        /** Update second order Butterworth low pass filter state with a new value.
         *
         * @param filter second order Butterworth low pass filter structure
         * @param value new input value of the filter
         * @return new filtered value
         */
        static inline float update_butterworth_2_low_pass(Butterworth2LowPass_t *filter, float value)
        {
            return update_second_order_low_pass((struct SecondOrderLowPass *)filter, value);
        }

        /** Get current value of the second order Butterworth low pass filter.
         *
         * @param filter second order Butterworth low pass filter structure
         * @return current value of the filter
         */
        static inline float get_butterworth_2_low_pass(Butterworth2LowPass_t *filter)
        {
            return filter->o[0];
        }

        /**
         * IIR filter the samples.
         */
        int16_t iirLPFilterSingle(int32_t in, int32_t attenuation,  int32_t* filt)
        {
            int32_t inScaled;
            int32_t filttmp = *filt;
            int16_t out;

            if (attenuation > (1<<IIR_SHIFT))
            {
                attenuation = (1<<IIR_SHIFT);
            }
            else if (attenuation < 1)
            {
                attenuation = 1;
            }

            // Shift to keep accuracy
            inScaled = in << IIR_SHIFT;
            // Calculate IIR filter
            filttmp = filttmp + (((inScaled-filttmp) >> IIR_SHIFT) * attenuation);
            // Scale and round
            out = (filttmp >> 8) + ((filttmp & (1 << (IIR_SHIFT - 1))) >> (IIR_SHIFT - 1));
            *filt = filttmp;

            return out;
        }

        void setCutoffFreq(const float sample_freq, const float cutoff_freq)
        {
            float fr = sample_freq/cutoff_freq;
            float ohm = tanf(M_PI/fr);
            float c = 1.0f+2.0f*cosf(M_PI/4.0f)*ohm+ohm*ohm;
            _b0 = ohm*ohm/c;
            _b1 = 2.0f*_b0;
            _b2 = _b0;
            _a1 = 2.0f*(ohm*ohm-1.0f)/c;
            _a2 = (1.0f-2.0f*cosf(M_PI/4.0f)*ohm+ohm*ohm)/c;
            _delay_element_1 = 0.0f;
            _delay_element_2 = 0.0f;
        }

    public:

        static const uint8_t IIR_SHIFT = 8;

        /**
         * 2-Pole low pass filter
         */
        void init(const float sample_freq, const float cutoff_freq)
        {
            if (cutoff_freq <= 0) {
                return;
            }

            setCutoffFreq(sample_freq, cutoff_freq);
        }

        float apply(float sample)
        {
            float delay_element_0 = sample - _delay_element_1 * _a1 - 
                _delay_element_2 * _a2;

            if (!isfinite(delay_element_0)) {
                // don't allow bad values to propigate via the filter
                delay_element_0 = sample;
            }

            float output = delay_element_0 * _b0 + _delay_element_1 * _b1 + 
                _delay_element_2 * _b2;

            _delay_element_2 = _delay_element_1;
            _delay_element_1 = delay_element_0;
            return output;
        }

}; // class Lpf
