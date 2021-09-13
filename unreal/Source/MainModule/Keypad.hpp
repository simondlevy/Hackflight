/*
   Numeric keypad control for flight axes

   Maintains doubleing-point values in [-1,+1]

   Copyright(C) 2021 Simon D.Levy

   MIT License
   */

#pragma once

#include "CoreMinimal.h"

class Keypad {

    private:

        static constexpr double STEP = .001;

        static const uint8_t AXES = 4;

        double _rawvals[AXES] = {};

        APlayerController * _playerController = NULL;

        bool hitEitherKey(const FKey key1, const FKey key2)
        {
            return hitKey(key1) || hitKey(key2);
        }

        bool hitKey(const FKey key)
        {
            return _playerController->IsInputKeyDown(key);
        }

        static const double max(double a, double b)
        {
            return a > b ? a : b;
        }

        static const double min(double a, double b)
        {
            return a < b ? a : b;
        }


        static void constrain(double & value, int8_t inc)
        {
            value += inc * STEP;

            value = value > +1 ? +1 : (value < -1 ? -1 : value);
        }

    public:

		Keypad(APawn * pawn)
		{
            _playerController =
                UGameplayStatics::GetPlayerController(pawn->GetWorld(), 0);
        }

        void tick(double * rawvals)
        {
            if (hitEitherKey(EKeys::Nine, EKeys::NumPadNine)) {
                constrain(_rawvals[0], +1);
            }

            if (hitEitherKey(EKeys::Three, EKeys::NumPadThree)) {
                constrain(_rawvals[0], -1);
            }

            if (hitEitherKey(EKeys::Six, EKeys::NumPadSix)) {
                constrain(_rawvals[1], +1);
            }

            if (hitEitherKey(EKeys::Four, EKeys::NumPadFour)) {
                constrain(_rawvals[1], -1);
            }

            if (hitEitherKey(EKeys::Eight, EKeys::NumPadEight)) {
                constrain(_rawvals[2], +1);
            }

            if (hitEitherKey(EKeys::Two, EKeys::NumPadTwo)) {
                constrain(_rawvals[2], -1);
            }

            if (hitKey(EKeys::Enter)) {
                constrain(_rawvals[3], +1);
            }

            if (hitEitherKey(EKeys::Zero, EKeys::NumPadZero)) {
                constrain(_rawvals[3], -1);
            }

            if (hitEitherKey(EKeys::Five, EKeys::NumPadFive)) {
                for (uint8_t i=0; i<AXES; ++i) {
                    _rawvals[i] = 0;
                }
            }

            for (uint8_t i=0; i<AXES; ++i) {
                rawvals[i] = _rawvals[i];
            }
        }

}; // class Keypad
