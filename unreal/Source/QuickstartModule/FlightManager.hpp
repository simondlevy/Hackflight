/*
   Simple MulticopterSim FlightManager class implementation

   Takes off to 10m altitude via PID control

   Copyright(C) 2021 Simon D.Levy

   MIT License
*/

#include "../MainModule/FlightManager.hpp"
#include "../MainModule/Dynamics.hpp"

#include "AltitudeController.hpp"

class FQuickstartFlightManager : public FFlightManager {

    private:

        AltitudeController _altitudeController;

        static constexpr double ALTITUDE_TARGET = 10;

        // Helps synchronize threads
        //bool _running = false;

    public:

        FQuickstartFlightManager(Dynamics * dynamics)
            : FFlightManager(dynamics)
        {
            //_running = true;
        }

        ~FQuickstartFlightManager()
        {
        }

        virtual void getActuators(const double time, double * values) override
        {
            /*
            if (!_running) {
                return;
            }*/

            float throttle = _altitudeController.getThrottle(
                    ALTITUDE_TARGET, 
                    time,
                    -_dynamics->x(Dynamics::STATE_Z),
                    -_dynamics->x(Dynamics::STATE_Z_DOT));

            for (uint8_t i=0; i<_dynamics->rotorCount(); ++i) {
                values[i] = throttle;
            }
        }

}; // FQuickstartFlightManager
