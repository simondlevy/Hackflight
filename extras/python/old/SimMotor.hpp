/*
   Hackflight Motor class implementation for MulticopterSim

   Copyright(C) 2020 Simon D.Levy

   MIT License
   */

#pragma once

#include <RFT_motor.hpp>

class SimMotor : public rft::Motor {

    private:

        float * _values = NULL;

    public:

        SimMotor(uint8_t count)
            : Motor(count)
        {
            _values = new float[count];
        }

        virtual ~SimMotor()
        {
            delete _values;
        }

        float getValue(uint8_t index)
        {
            return _values[index];
        }

    protected:

        virtual void write(uint8_t index, float value) override
        {
            _values[index] = value;
        }

}; // class SimMotor
