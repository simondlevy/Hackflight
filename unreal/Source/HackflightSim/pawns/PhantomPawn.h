/*
* Class declaration for DJI Phantom pawn class using Haskell 
*
* Copyright (C) 2019 Simon D. Levy
*
* MIT License
*/

#pragma once

#include "../Vehicle.hpp"
#include "../DynamicsThread.h"
#include "../dynamics/fixedpitch/QuadXAP.hpp"

#include <CoreMinimal.h>
#include <GameFramework/Pawn.h>

#include "PhantomPawn.generated.h"

// Structures to hold static mesh initializations
DECLARE_STATIC_MESH(FFrameStatics, "Phantom/Frame.Frame", FrameStatics)
DECLARE_STATIC_MESH(FPropCWStatics, "Phantom/PropCW.PropCW", PropCWStatics)
DECLARE_STATIC_MESH(FPropCCWStatics, "Phantom/PropCCW.PropCCW", PropCCWStatics)

UCLASS(Config=Game)
class APhantomPawn : public APawn {

    private:

        GENERATED_BODY()

        Dynamics::vehicle_params_t vparams = {

            // Estimated
            2.E-06, // d drag cofficient [T=d*w^2]

            // https://www.dji.com/phantom-4/info
            1.380,  // m mass [kg]

            // Estimated
            2,      // Ix [kg*m^2] 
            2,      // Iy [kg*m^2] 
            3,      // Iz [kg*m^2] 
            38E-04, // Jr prop inertial [kg*m^2] 
            15000,// maxrpm
        };

        FixedPitchDynamics::fixed_pitch_params_t fparams = {
            5.E-06, // b thrust coefficient [F=b*w^2]
            0.350   // l arm length [m]
        };

        FDyanmicsThread * _flightManager = NULL;

        void addRotor(UStaticMesh * mesh, int8_t dx, int8_t dy);

    protected:

        // AActor overrides

        virtual void BeginPlay() override;

        virtual void Tick(float DeltaSeconds) override;

        virtual void PostInitializeComponents() override;

        virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

        // virtual void NotifyHit(...) override

    public:	

        QuadXAPDynamics dynamics = QuadXAPDynamics(vparams, fparams);

        Vehicle vehicle = Vehicle(&dynamics);

        APhantomPawn();

}; // APhantomPawn
