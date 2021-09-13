/*
   MulticopterSim FlightManager class implementation using Haskell Copilot

   Copyright(C) 2021 Simon D.Levy

   MIT License
*/

#include "../MainModule/FlightManager.hpp"
#include "../MainModule/Dynamics.hpp"
#include "../MainModule/GameInput.hpp"

class FCopilotFlightManager : public FFlightManager {

    private:

        // Joystick (RC transmitter, game controller) or keypad
        GameInput * _gameInput = NULL;
        double _joyvals[4] = {};

        // For guarding thread
        bool _ready = false;

        // Helpers
        void getReceiverDemands(void);
        void getGyrometer(void);
        void getQuaternion(void);
        void getOpticalFlow(void);

    public:

        FCopilotFlightManager(APawn * pawn, Dynamics * dynamics);
		
        ~FCopilotFlightManager();

        virtual void getActuators(const double time, double * values) override;

        void tick(void);

}; // FCopilotFlightManager
