/*
 * Support for joysticks and other game controllers
 *
 * We'd like this to be an .hpp file, but UE4 requires it
 * to be .h
 *
 * Copyright (C) 2021 Simon D. Levy
 *
 * MIT License
 */


#pragma once

#include <CoreMinimal.h>
#include <UObject/Interface.h>
#include "Joystick.generated.h"

// This class does not need to be modified.
UINTERFACE(MinimalAPI)
class UJoystick : public UInterface
{
	GENERATED_BODY()
};

/**
 * 
 */
class IJoystick
{
	GENERATED_BODY()

    private:

        static const uint16_t PRODUCT_XBOX_ONE         = 0x02ff;
        static const uint16_t PRODUCT_XBOX360          = 0x02a1;
        static const uint16_t PRODUCT_XBOX360_CLONE    = 0xfafe;
        static const uint16_t PRODUCT_XBOX360_CLONE2   = 0x028e;
        static const uint16_t PRODUCT_XBOX360_WIRELESS = 0x0719;
        static const uint16_t PRODUCT_TARANIS_QX7      = 0x5720;
        static const uint16_t PRODUCT_TARANIS_X9D      = 0x5710;
        static const uint16_t PRODUCT_PS3_CLONE        = 0x0003;
        static const uint16_t PRODUCT_INTERLINK        = 0x0e56;
        static const uint16_t PRODUCT_SPEKTRUM         = 0x572b;
        static const uint16_t PRODUCT_EXTREMEPRO3D     = 0xc215;
        static const uint16_t PRODUCT_F310             = 0xc216;
        static const uint16_t PRODUCT_PS4              = 0x09cc;

        static constexpr double AUX1_MID = 0.3f; // positve but less than 0.5

        uint16_t _productId = 0;

        int _joystickId = 0;

        bool _isGameController = false;

        static void getAxes4(double axes[6],
                             DWORD axis0,
                             DWORD axis1,
                             DWORD axis2,
                             DWORD axis3)
        {
            axes[0] = (double)axis0;
            axes[1] = (double)axis1;
            axes[2] = (double)axis2;
            axes[3] = (double)axis3;
        }

        static void getAxes5(double axes[6],
                             uint8_t & naxes,
                             DWORD axis0,
                             DWORD axis1,
                             DWORD axis2,
                             DWORD axis3,
                             DWORD axis4)
        {
            naxes = 5;
            getAxes4(axes, axis0, axis1, axis2, axis3);
            axes[4] = (double)axis4;
        }

    protected:

        enum {

            AX_THR,
            AX_ROL,
            AX_PIT,
            AX_YAW,
            AX_AU1,
            AX_AU2,
            AX_NIL
        };

        void buttonsToAxes(uint8_t buttons,
                           uint8_t top,
                           uint8_t rgt,
                           uint8_t bot,
                           uint8_t lft,
                           double * axes)
        {
            static double _aux1 = 0;
            static double _aux2 = -1;

            static bool _down;

            if (buttons) {

                if (!_down) {

                    // Left button sets AUX2
                    if (buttons == lft) {
                        _aux2 *= -1;
                    }

                    // Other buttons set AUX1
                    else {
                        _aux1 = (buttons == top) ?
                                -1 :
                                (buttons == rgt ? AUX1_MID : +1);
                    }

                    _down = true;
                }
            }

            else {
                _down = false;
            }

            axes[AX_AU1] = _aux1;
            axes[AX_AU2] = _aux2;
        }

        static bool isValidJoystick(int joystick_id, uint16_t & product_id);

        static void readJoystick(
                int joystick_id,
                uint32_t & xpos,
                uint32_t & ypos,
                uint32_t & zpos, 
                uint32_t & rpos,
                uint32_t & upos,
                uint32_t & vpos, 
                uint8_t & buttons);

    public:

        IJoystick(void)
        {
            _isGameController = false;

            // Grab the first available joystick
            for (_joystickId=0; _joystickId<16; _joystickId++) {
                if (isValidJoystick(_joystickId, _productId)) {
                    break;
                }
            }

            if (_joystickId < 16) {

                switch (_productId) {
                    case PRODUCT_TARANIS_QX7:
                    case PRODUCT_TARANIS_X9D:
                    case PRODUCT_SPEKTRUM:
                        _isGameController = false;
                        break;
                    default: 
                        _isGameController = true;
                }
            }
        }

        void poll(double axes[6])
        {
            uint32_t xpos = 0; 
            uint32_t ypos = 0;
            uint32_t zpos = 0;
            uint32_t rpos = 0;
            uint32_t upos = 0;
            uint32_t vpos = 0; 

            uint8_t buttons = 0;

            readJoystick(_joystickId,
                    xpos, ypos, zpos, rpos, upos, vpos, buttons);

            // axes: 0=Throttle 1=Roll 2=Pitch 3=Yaw 4=Aux

            uint8_t naxes = 4;

            switch (_productId) {

                case PRODUCT_SPEKTRUM:
                    getAxes5(axes, naxes, ypos, zpos, vpos, xpos, upos);
                    break;

                case PRODUCT_TARANIS_QX7:
                case PRODUCT_TARANIS_X9D:
                    getAxes5(axes, naxes, xpos, ypos, zpos, vpos, rpos);
                    break;

                case PRODUCT_PS3_CLONE:      
                case PRODUCT_PS4:
                    getAxes4(axes, ypos, zpos, rpos, xpos);
                    break;

                case PRODUCT_F310:
                    getAxes4(axes, ypos, zpos, rpos, xpos);
                    break;

                case PRODUCT_XBOX_ONE:
                case PRODUCT_XBOX360:
                case PRODUCT_XBOX360_CLONE:
                case PRODUCT_XBOX360_CLONE2:
                case PRODUCT_XBOX360_WIRELESS:
                    getAxes4(axes, ypos, upos, rpos, xpos);
                    break;

                case PRODUCT_EXTREMEPRO3D:  
                    getAxes4(axes, zpos, xpos, ypos, rpos);
                    break;

                default: // failed
                    return;
            }

            // Normalize the axes to demands to [-1,+1]
            for (uint8_t k=0; k<naxes; ++k) {
                axes[k] = axes[k] / 32767 - 1;
            }

            // Invert throttle, pitch axes on game controllers
            if (_isGameController) {
                axes[AX_THR] *= -1;
                axes[AX_PIT] *= -1;
            }

            switch (_productId) {

                case PRODUCT_F310:
                    buttonsToAxes(buttons, 8, 4, 2, 1, axes);
                    break;

                case PRODUCT_XBOX_ONE:
                case PRODUCT_XBOX360:
                case PRODUCT_XBOX360_CLONE:
                case PRODUCT_XBOX360_CLONE2:
                case PRODUCT_XBOX360_WIRELESS:
                    buttonsToAxes(buttons, 8, 2, 1, 4, axes);
            }
        }
};
