/*
   Input via joystick and/or keypad

   Joystick runs on its own thread; keypad runs on
   main thread

   Copyright(C) 2021 Simon D.Levy

   MIT License
   */

#pragma once

#include "Joystick.h"
#include "Keypad.hpp"

class GameInput {

		IJoystick * _joystick = NULL;

        Keypad * _keypad = NULL;

 public:

		GameInput(APawn * pawn)
		{
			_joystick = new IJoystick();

            _keypad = new Keypad(pawn);
		}

		void getJoystick(double * rawvals)
		{
		    _joystick->poll(rawvals);
 		}

        void getKeypad(double * rawvals)
        {
            _keypad->tick(rawvals);
        }

}; // class GameInput
