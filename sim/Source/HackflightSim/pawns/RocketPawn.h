/*
* Class declaration for thrust-vectoring rocket class using Haskell 
*
* Copyright (C) 2021 Simon D. Levy
*
* MIT License
*/

#pragma once

#include "../Vehicle.hpp"
#include "../DynamicsThread.h"

#include <CoreMinimal.h>
#include <GameFramework/Pawn.h>

#include "RocketPawn.generated.h"

// Structures to hold static mesh initializations
DECLARE_STATIC_MESH(FBodyStatics, "Rocket/Body.Body", BodyStatics)
DECLARE_STATIC_MESH(FRotorTopStatics, "Rocket/RotorTop.RotorTop", RotorTopStatics)
DECLARE_STATIC_MESH(FRotorBottomStatics, "Rocket/RotorBottom.RotorBottom", RotorBottomStatics)
DECLARE_STATIC_MESH(FNozzleStatics, "Rocket/Nozzle.Nozzle", NozzleStatics)

UCLASS(Config=Game)
class ARocketPawn : public APawn {

    private:

        GENERATED_BODY()

        static constexpr double ROTOR_TOP_Z    =  0.70;
        static constexpr double ROTOR_BOTTOM_Z =  0.60;

        FDynamicsThread * dynamicsThread = NULL;

        void addRotor(UStaticMesh* mesh, float z, int8_t dir);

        float meshHeightMeters(UStaticMesh * mesh);

        // A private class to support animating the nozzle
        class NozzleVehicle : public Vehicle {

            friend class ARocketPawn;

            static constexpr float NOZZLE_Z =  0.15;

            // XXX should come from Haskell dynamics
            static constexpr float NOZZLE_MAX_ANGLE = 90; //45; 

            ARocketPawn * pawn = NULL;

            UStaticMeshComponent * nozzleMeshComponent = NULL;

            virtual void animateActuators(void) override;

        };

    protected:

        // AActor overrides

        virtual void BeginPlay() override;

        virtual void Tick(float DeltaSeconds) override;

        virtual void PostInitializeComponents() override;

        virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    public:	

        NozzleVehicle vehicle;

        ARocketPawn();

}; // ARocketPawn
