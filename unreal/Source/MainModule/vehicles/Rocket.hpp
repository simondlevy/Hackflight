/*
 Helper class for pawns using rocket frame
 *
 * Copyright (C) 2020 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include "../Vehicle.hpp"

#include "../dynamics/ThrustVector.hpp"

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

// Structures to hold static mesh initializations
DECLARE_STATIC_MESH(FBodyStatics, "Rocket/Body.Body", BodyStatics)
DECLARE_STATIC_MESH(FRotorTopStatics, "Rocket/RotorTop.RotorTop", RotorTopStatics)
DECLARE_STATIC_MESH(FRotorBottomStatics, "Rocket/RotorBottom.RotorBottom", RotorBottomStatics)
DECLARE_STATIC_MESH(FNozzleStatics, "Rocket/Nozzle.Nozzle", NozzleStatics)

class Rocket {

    private:

        Dynamics::vehicle_params_t vparams = {

            // Estimated
            2.E-06, // d drag coefficient [T=d*w^2]

            // https://www.dji.com/phantom-4/info
            1.380,  // m mass [kg]

            // Estimated
            2,      // Ix [kg*m^2] 
            2,      // Iy [kg*m^2] 
            3,      // Iz [kg*m^2] 
            38E-04, // Jr prop inertial [kg*m^2] 
            15000,  // maxrpm
        };

        // Affects dynamics
        static constexpr double NOZZLE_MAX_ANGLE =  45;
        static constexpr double NOZZLE_Z         =  0.15;

        // For appearance only
        static constexpr double ROTOR_TOP_Z  =  0.70;
        static constexpr double ROTOR_BOTTOM_Z  =  0.60;

        // A private class to support animating the nozzle
        class NozzleVehicle : public Vehicle {

            friend class Rocket;

            UStaticMeshComponent * nozzleMeshComponent = NULL;

            NozzleVehicle(Dynamics* dynamics) 
                : Vehicle(dynamics)
            {
            }

            virtual void animateActuators(void) override
            {
                Vehicle::animateActuators();

                nozzleMeshComponent->SetRelativeRotation(FRotator(-_flightManager->actuatorValue(3)*NOZZLE_MAX_ANGLE,
                                                                  0,
                                                                  -_flightManager->actuatorValue(2)*NOZZLE_MAX_ANGLE));
            }
        };

        // Threaded worker for flight control
        FFlightManager * _flightManager = NULL;

        void addRotor(UStaticMesh* mesh, float z)
        {
            _vehicle->addRotor(mesh, 0, 0, z);
        }

        float meshHeightMeters(UStaticMesh * mesh) 
        {
            FBox box = mesh->GetBoundingBox();

            return (box.Max.Z - box.Min.Z) / 100; // cm => m
        }

        NozzleVehicle * _vehicle = NULL;

    public:

        ThrustVectorDynamics dynamics = ThrustVectorDynamics(vparams, NOZZLE_MAX_ANGLE);


        void build(APawn * pawn)
        {
            // Get height of barrel for dynamics
            float barrelHeight = meshHeightMeters(BodyStatics.mesh.Get());

            // Create vehicle object from dynamics
            _vehicle = new NozzleVehicle(&dynamics);

            // Add barrel mesh to vehicle
            _vehicle->buildFull(pawn, BodyStatics.mesh.Get());

            // Add rotors
            addRotor(RotorTopStatics.mesh.Get(), ROTOR_TOP_Z);
            addRotor(RotorBottomStatics.mesh.Get(), ROTOR_BOTTOM_Z);

            // Add nozzle
            _vehicle->nozzleMeshComponent =
                _vehicle->addComponent(NozzleStatics.mesh.Get(),
                                       FName("Nozzle"), 0, 0, NOZZLE_Z, 0);

            _flightManager = NULL;
        }

        void PostInitializeComponents()
        {
            _vehicle->PostInitializeComponents();
        }

        void BeginPlay(FFlightManager * flightManager)
        {
            _flightManager = flightManager;

            _vehicle->BeginPlay(flightManager);
        }

        void EndPlay(void)
        {
            FFlightManager::stopThread(&_flightManager);
        }

        void Tick(float DeltaSeconds)
        {
            _vehicle->Tick(DeltaSeconds);
        }

        void addCamera(Camera * camera)
        {
            _vehicle->addCamera(camera);
        }


}; // class Rocket 
