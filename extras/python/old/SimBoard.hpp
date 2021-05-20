/*
   Hackflight Board class implementation for MulticopterSim

   Copyright(C) 2019 Simon D.Levy

   MIT License
*/

#pragma once

// #include <RFT_board.hpp>

class SimBoard : public rft::Board {

    private:
   
        float _currentTime = 0; // must be float for Hackflight

    protected:

        float getTime(void)
        {
            return _currentTime;
        }

    public:

        SimBoard() 
        {
        }

        virtual ~SimBoard() 
		{ 
		}

        void set(const double time)
        {
            _currentTime = time;
        }

}; // class Simboard

