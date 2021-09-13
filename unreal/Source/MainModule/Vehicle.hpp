/*
 * General support for vehicles in MulticopterSim
 *
 * This class peforms the following functions:
 *
 * (1) Statically builds meshes, cameras, and other UE4 objects
 *
 * (2) Provides basic support for displaying vehicle kinematics
 *
 * Copyright (C) 2019 Simon D. Levy, Daniel Katzav
 *
 * MIT License
 */

#pragma once

#define WIN32_LEAN_AND_MEAN

#include "Runtime/Landscape/Classes/Landscape.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"

#include "Utils.hpp"
#include "Dynamics.hpp"
#include "FlightManager.hpp"
#include "Camera.hpp"

#include <stdio.h>

#define SPRINTF sprintf_s

// A macro for simplifying the declaration of static meshes
#define DECLARE_STATIC_MESH(structname, assetstr, objname)   \
    struct structname {                                             \
        ConstructorHelpers::FObjectFinderOptional<UStaticMesh> mesh;   \
        structname() : mesh(TEXT("/Game/MulticopterSim/Meshes/" assetstr)) { } \
    };                                                                     \
    static structname objname;


class Vehicle {

    private:

        // Support cycling through views by hitting spacebar
        typedef enum {

            VIEW_CHASE,
            VIEW_FRONT, 
            VIEW_GROUND,
            VIEW_COUNT

        } view_t;
        view_t _view = VIEW_CHASE;

        uint8_t _nrotors = 0;

        float rotorStartAngle(float rotorX, float rotorY)
        {
            FVector vehicleCenter = _pawn->GetActorLocation();
            double theta = -atan2((rotorY - vehicleCenter.Y),
                                  (rotorX - vehicleCenter.X));
            return FMath::RadiansToDegrees(M_PI / 2 - theta) + 57.5;
        }

        void rotateRotors(int8_t* rotorDirections)
        {
            static float rotation;
            for (uint8_t i = 0; i < _rotorCount; ++i) {
                setRotorRotation(i, rotation * rotorDirections[i] * 200);
            }
            rotation++;
        }

        bool _mapSelected = false;

        // Special camera for ground view
        ACameraActor* _groundCamera = NULL;

        // Useful approximation to infinity for tracing rays
        static constexpr float INF = 1e9;

        // Time during which velocity will be set to zero during final phase
        // oflanding
        static constexpr float SETTLING_TIME = 1.0;

        // UE4 objects that must be built statically
        UStaticMesh* _frameMesh = NULL;
        USoundCue* _soundCue = NULL;
        USpringArmComponent* _gimbalSpringArm = NULL;
        USpringArmComponent * _playerCameraSpringArm = NULL;
        USpringArmComponent* _bodyHorizontalSpringArm = NULL;
        UCameraComponent* _playerCamera = NULL;

        // Support for switching from chase camera to FPV
        float _playerCameraFollowMeters = 0;
        float _playerCameraElevationMeters = 0;

        // PlayerController for getting keyboard events
        APlayerController * _playerController = NULL;

        // Cameras
        Camera* _cameras[Camera::MAX_CAMERAS];
        uint8_t  _cameraCount;

        // For computing AGL
        float _aglOffset = 0;

        // Countdown for zeroing-out velocity during final phase of landing
        float _settlingCountdown = 0;

        // Starting location, for kinematic offset
        FVector _startLocation = {};

        // Retrieves kinematics from dynamics computed in another thread
        void updateKinematics(void)
        {
            // Set vehicle pose in animation
            _pawn->SetActorLocation(_startLocation +
                100 * FVector(_dynamics->x(Dynamics::STATE_X), 
                              _dynamics->x(Dynamics::STATE_Y),
                              -_dynamics->x(Dynamics::STATE_Z))); // for NED

            _pawn->SetActorRotation(
                    FMath::RadiansToDegrees(
                        FRotator(_dynamics->x(Dynamics::STATE_THETA),
                            _dynamics->x(Dynamics::STATE_PSI),
                            _dynamics->x(Dynamics::STATE_PHI))));
        }

        void grabImages(void)
        {
            for (uint8_t i = 0; i < _cameraCount; ++i) {
                _cameras[i]->grabImage();
            }
        }

        void buildPlayerCameras(float distanceMeters, float elevationMeters)
        {
            _bodyHorizontalSpringArm =
                _pawn->CreateDefaultSubobject<USpringArmComponent>(
                        TEXT("BodyHorizontalSpringArm"));
            _bodyHorizontalSpringArm->SetupAttachment(_frameMeshComponent);
            _bodyHorizontalSpringArm->SetRelativeLocationAndRotation(
                    FVector::ZeroVector, FRotator::ZeroRotator);
            _bodyHorizontalSpringArm->TargetArmLength = 0;
            _bodyHorizontalSpringArm->bEnableCameraLag = false;
            _bodyHorizontalSpringArm->bInheritPitch = false;
            _bodyHorizontalSpringArm->bInheritRoll = false;

            _playerCameraFollowMeters = distanceMeters;
            _playerCameraElevationMeters = elevationMeters;

            _playerCameraSpringArm =
                _pawn->CreateDefaultSubobject<USpringArmComponent>(
                        TEXT("PlayerCameraSpringArm"));
            _playerCameraSpringArm->SetupAttachment(_bodyHorizontalSpringArm);

            _playerCameraSpringArm->bEnableCameraLag = false;
            _playerCameraSpringArm->bInheritYaw = true;
            _playerCameraSpringArm->bInheritPitch = false;
            _playerCameraSpringArm->bInheritRoll = false;
            _playerCameraSpringArm->bEnableCameraRotationLag = true;

            _playerCamera =
                _pawn->CreateDefaultSubobject<UCameraComponent>(
                        TEXT("PlayerCamera"));
            _playerCamera->SetupAttachment(_playerCameraSpringArm,
                                          USpringArmComponent::SocketName);
        }


        void setView()
        {
            if (!_playerCameraSpringArm) {
                return;
            }

            switch (_view) {
            case VIEW_FRONT:
                _playerController->SetViewTargetWithBlend(_pawn);
                _playerCameraSpringArm->SetRelativeLocationAndRotation(FVector::ZeroVector, FRotator::ZeroRotator);

                // empircally determined to be far enough ahead of vehicle
                _playerCameraSpringArm->TargetArmLength = -30; 

                _bodyHorizontalSpringArm->bInheritYaw = true;
                break;
            case VIEW_GROUND:
                if (_groundCamera) {
                    _playerController->SetViewTargetWithBlend(_groundCamera);
                }
                break;
            default:
                _playerController->SetViewTargetWithBlend(_pawn);
                _playerCameraSpringArm->SetRelativeLocationAndRotation(
                        FVector(-_playerCameraFollowMeters, 0, 
                                _playerCameraElevationMeters) * 100,
                    FRotator::ZeroRotator);;
                _playerCameraSpringArm->TargetArmLength =
                    _playerCameraFollowMeters * 100;

                _bodyHorizontalSpringArm->bInheritYaw = false;
            }
        }

    protected:

        UAudioComponent* _audioComponent = NULL;

        // Set in constructor
        Dynamics* _dynamics = NULL;

        APawn* _pawn = NULL;

        UStaticMeshComponent* _frameMeshComponent = NULL;

	    UStaticMeshComponent* _rotorMeshComponents[Dynamics::MAX_ROTORS] = {};

        // Threaded worker for running flight control
        class FFlightManager* _flightManager = NULL;

        // Circular buffer for moving average of rotor values
        TCircularBuffer<float>* _rotorBuffer = NULL;
        uint32_t _bufferIndex = 0;

        // Starts at zero and increases each time we add a rotor
        uint8_t _rotorCount;

        // Also set in constructor, but purely for visual effect
        int8_t _rotorDirections[Dynamics::MAX_ROTORS] = {};

        virtual void animateActuators(void)
        {
            // Compute the sum of the rotor values
            float rotorSum = 0;
            for (uint8_t j = 0; j < _nrotors; ++j) {
                rotorSum += _flightManager->actuatorValue(j);
            }

            // Rotate rotors. For visual effect, we can ignore actual rotor
            // values, and just keep increasing the rotation.
            if (rotorSum > 0) {
                rotateRotors(_rotorDirections);
            }

            // Add mean to circular buffer for moving average
            _bufferIndex = _rotorBuffer->GetNextIndex(_bufferIndex);
            (*_rotorBuffer)[_bufferIndex] = rotorSum / _nrotors;

            // Compute the mean rotor value over the buffer frames
            float smoothedRotorMean = 0;
            for (uint8_t i = 0; i < _rotorBuffer->Capacity(); ++i) {
                smoothedRotorMean += (*_rotorBuffer)[i];
            }
            smoothedRotorMean /= _rotorBuffer->Capacity();

            // Use the mean rotor value to modulate the pitch and voume of the
            // rotor sound
            if (_audioComponent) {

                _audioComponent->SetFloatParameter(FName("pitch"),
                                                   smoothedRotorMean);
                _audioComponent->SetFloatParameter(FName("volume"),
                                                   smoothedRotorMean);
            }
        }


    public:

        void build(APawn* pawn, UStaticMesh* frameMesh)
        {
            _pawn = pawn;
            _frameMesh = frameMesh;

            _frameMeshComponent =
                _pawn->CreateDefaultSubobject<UStaticMeshComponent>(
                        TEXT("FrameMesh"));
            _frameMeshComponent->SetStaticMesh(_frameMesh);
            _frameMeshComponent->SetCollisionResponseToAllChannels(
                    ECollisionResponse::ECR_Overlap);
            
            _pawn->SetRootComponent(_frameMeshComponent);

            _rotorCount = 0;
        }

        void buildFull(APawn* pawn,
                       UStaticMesh* frameMesh,
                       float chaseCameraDistanceMeters=1.5, 
                float chaseCameraElevationMeters=0.5)
        {
            build(pawn, frameMesh);

            // Build the player-view cameras
            buildPlayerCameras(chaseCameraDistanceMeters,
                               chaseCameraElevationMeters);

            // Get sound cue from Contents
            static ConstructorHelpers::FObjectFinder<USoundCue> soundCue(
                    TEXT("/Game/MulticopterSim/Audio/MotorSoundCue"));

            // Store a reference to the Cue asset - we'll need it later.
            _soundCue = soundCue.Object;

            // Create an audio component, which wraps the sound cue, and allows
            // us to ineract with it and its parameters from code
            _audioComponent =
                _pawn->CreateDefaultSubobject<UAudioComponent>(
                        TEXT("RotorAudioComp"));

            if (_audioComponent) {

                // Set the audio component's volume to zero
                _audioComponent->SetFloatParameter(FName("volume"), 0);

                // Attach the sound to the pawn's root, the sound follows the
                // pawn around
                _audioComponent->SetupAttachment(_pawn->GetRootComponent());
            }

            // Create a spring-arm for the gimbal
            _gimbalSpringArm =
                _pawn->CreateDefaultSubobject<USpringArmComponent>(
                        TEXT("GimbalSpringArm"));
            _gimbalSpringArm->SetupAttachment(_pawn->GetRootComponent());
            _gimbalSpringArm->TargetArmLength = 0.f;
        }

        void addMesh(UStaticMesh* mesh,
                     const char* name,
                     const FVector& location,
                     const FRotator rotation,
                     const FVector& scale)
        {
            UStaticMeshComponent* meshComponent =
                _pawn->CreateDefaultSubobject<UStaticMeshComponent>(
                        FName(name));
            meshComponent->SetStaticMesh(mesh);
            meshComponent->SetupAttachment(_frameMeshComponent,
                                           USpringArmComponent::SocketName);
            meshComponent->AddRelativeLocation(location * 100); // m => cm
            meshComponent->AddLocalRotation(rotation);
            meshComponent->SetRelativeScale3D(scale);
        }

        void addMesh(UStaticMesh* mesh,
                     const char* name,
                     const FVector& location,
                     const FRotator rotation)
        {
            addMesh(mesh, name, location, rotation, FVector(1, 1, 1));
        }

        void addMesh(UStaticMesh* mesh, const char* name)
        {
            addMesh(mesh, name, FVector(0, 0, 0), FRotator(0, 0, 0));
        }

        void addCamera(Camera* camera)
        {
            // Add camera to spring arm
            camera->addToVehicle(_pawn, _gimbalSpringArm, _cameraCount);

            // Increment the camera count for next time
            _cameras[_cameraCount++] = camera;
        }

        Vehicle(void)
        {
            _dynamics = NULL;
            _flightManager = NULL;
        }

        Vehicle(Dynamics* dynamics)
        {
            _dynamics = dynamics;
            _nrotors = dynamics->rotorCount();

            for (uint8_t i = 0; i < dynamics->rotorCount(); ++i) {
                _rotorDirections[i] = dynamics->getRotorDirection(i);
            }

            _flightManager = NULL;
        }

        virtual ~Vehicle(void)
        {
        }

        void BeginPlay(FFlightManager* flightManager)
        {
            _flightManager = flightManager;

            // Player controller is useful for getting keyboard events,
            // switching cameas, etc.
            _playerController =
                UGameplayStatics::GetPlayerController(_pawn->GetWorld(), 0);

            // Change view to player camera on start
            _playerController->SetViewTargetWithBlend(_pawn);


            // Check landscape for world parameters
            for (TActorIterator<ALandscape> LandscapeItr(_pawn->GetWorld());
                 LandscapeItr;
                 ++LandscapeItr) {

                for (FName Tag : LandscapeItr->Tags) {

                    FString tag = Tag.ToString();
                    if (tag.Contains("g=") && tag.Contains("rho=")) {
                        float g = 0, rho = 0;
                        if (sscanf_s(TCHAR_TO_ANSI(*tag),
                                    "g=%f rho=%f", &g, &rho) == 2) {
                            _dynamics->setWorldParams(g, rho);
                        }
                    }
                }
            }

            // Make sure a map has been selected
            _mapSelected = false;
            FString mapName = _pawn->GetWorld()->GetMapName();
            if (mapName.Contains("Untitled")) {
                error("NO MAP SELECTED");
                return;
            }
            _mapSelected = true;

            // Disable built-in physics
            _frameMeshComponent->SetSimulatePhysics(false);

            // Start the audio for the rotors Note that because the Cue Asset
            // is set to loop the sound, once we start playing the sound, it
            // will play continiously...
            if (_audioComponent) {
                _audioComponent->Play();
            }

            // Create circular queue for moving-average of rotor values
            _rotorBuffer = new TCircularBuffer<float>(20);

            // Get vehicle ground-truth location for kinematic offset
            _startLocation = _pawn->GetActorLocation();

            // AGL offset will be set to a positve value the first time agl()
            // is called
            _aglOffset = 0;

            // Get vehicle ground-truth rotation to initialize flight manager
            FRotator startRotation = _pawn->GetActorRotation();

            // Initialize dynamics with initial rotation
            double rotation[3] = {
                FMath::DegreesToRadians(startRotation.Roll),
                FMath::DegreesToRadians(startRotation.Pitch),
                FMath::DegreesToRadians(startRotation.Yaw) };
            _dynamics->init(rotation);

            // Find the first cine camera in the viewport
            _groundCamera = NULL;
            for (TActorIterator<ACameraActor> cameraItr(_pawn->GetWorld());
                 cameraItr;
                 ++cameraItr) {

                ACameraActor * cameraActor = *cameraItr;
                if (cameraActor->GetName().StartsWith("CineCamera")) {
                    _groundCamera = cameraActor;
                }
            }

            _view = VIEW_CHASE;
            setView();
        }

        void Tick(float DeltaSeconds)
        {
            // Quit on ESCape key
            if (hitKey(EKeys::Escape)) {
                RequestEngineExit("User hit ESC");
            }

            // Run the game if a map has been selected
            if (_mapSelected) {

                // Use spacebar to switch player-camera view
                setPlayerCameraView();

                updateKinematics();

                grabImages();

                animateActuators();

                _dynamics->setAgl(agl());
            }
        }

        bool hitKey(const FKey key)
        {
            return _playerController->IsInputKeyDown(key);
        }

        void setPlayerCameraView(void)
        {
            if (_groundCamera) {
                _groundCamera->SetActorRotation(
                        UKismetMathLibrary::FindLookAtRotation(
                            _groundCamera->GetActorLocation(),
                            _pawn->GetActorLocation()));
            }

            // avoid registering multiple spacebar presses
            static bool didhit;

            if (hitKey(EKeys::SpaceBar)) {
                if (!didhit) {
                    switch (_view) {
                        case VIEW_CHASE:
                            _view = VIEW_FRONT;
                            break;
                         case VIEW_FRONT:
                            _view = VIEW_GROUND;
                            break;
                        case VIEW_GROUND:
                            _view = VIEW_CHASE;
                            break;
                    }
                    setView();
                }
                didhit = true;
            }
            else {
                didhit = false;
            }
        }


        // Returns AGL when vehicle is level above ground, "infinity" otherwise
        float agl(void)
        {
            // Start at the center of the vehicle
            FVector startPoint = _pawn->GetActorLocation();
            startPoint.Z += 100;
            // End at a point an "infinite" distance below the start point
            FVector endPoint = FVector(startPoint.X,
                                       startPoint.Y,
                                       startPoint.Z - INF);

            float d = getImpactDistance(startPoint, endPoint);

            // The first time we measure, we need to set the offset
            if (_aglOffset == 0) {
                _aglOffset = d;
            }

            return d - _aglOffset;
        }

        // Returns distance to mesh between points, or -1 if none found.
        // Eventually we may want to be able to specifiy an actor or actors to
        // include or exclude (other than the vehicle itself).
        float getImpactDistance(FVector startPoint, FVector endPoint)
        {
            // Currently, the only collisions we ignore are with the pawn
            // itself
            TArray<AActor*> actorsToIgnore;
            actorsToIgnore.Add(_pawn);
            FCollisionQueryParams traceParams(FName(TEXT("Distance Trace")),
                                              true, 
                                              actorsToIgnore[0]);
            traceParams.AddIgnoredActors(actorsToIgnore);

            FHitResult OutHit;
            if (_pawn->GetWorld()->LineTraceSingleByChannel(OutHit,
                        startPoint, endPoint, ECC_Visibility, traceParams)) {
                if (OutHit.bBlockingHit) {
                    FVector impactPoint = OutHit.ImpactPoint;
                    return (startPoint.Z - impactPoint.Z) / 100;
                }
            }

            return -1;
        }

        void drawHorizontal(FVector point)
        {
            FVector lftPoint = FVector(point.X, point.Y - 100, point.Z);
            FVector rgtPoint = FVector(point.X, point.Y + 100, point.Z);
            drawLine(lftPoint, rgtPoint);
        }

        void drawLine(FVector point1, FVector point2)
        {
            DrawDebugLine(_pawn->GetWorld(), point1, point2, FColor::Green,
                          false, .1,0, 0.5);
        }

        void PostInitializeComponents()
        {
            // Add "Vehicle" tag for use by level blueprint
            _pawn->Tags.Add(FName("Vehicle"));

            if (_audioComponent && _soundCue->IsValidLowLevelFast()) {
                _audioComponent->SetSound(_soundCue);
            }
        }

        void rotateGimbal(FQuat rotation)
        {
            _gimbalSpringArm->SetRelativeRotation(rotation);
        }


        UStaticMeshComponent* getFrameMesh(void)
        {
            return _frameMeshComponent;
        }

        UStaticMeshComponent * addComponent(UStaticMesh * mesh,
                                            FName name,
                                            float x=0,
                                            float y=0,
                                            float z=0,
                                            float yaw_angle=0)
        {
            UStaticMeshComponent* meshComponent =
                _pawn->CreateDefaultSubobject<UStaticMeshComponent>(name);
            meshComponent->SetStaticMesh(mesh);
            meshComponent->SetupAttachment(_frameMeshComponent,
                                           USpringArmComponent::SocketName);

            // m => cm
            meshComponent->AddRelativeLocation(FVector(x, y, z) * 100);

            meshComponent->SetRelativeRotation(FRotator(0, yaw_angle, 0));

            return meshComponent;
        }

        UStaticMeshComponent * addRotor(UStaticMesh* rotorMesh,
                float x, float y, float z, float angle)
        {
            UStaticMeshComponent * rotorMeshComponent =
                addComponent(rotorMesh, makeName("Rotor", _rotorCount, "Mesh"),
                        x, y, z, angle);
            _rotorMeshComponents[_rotorCount] = rotorMeshComponent;
            _rotorCount++;
            return rotorMeshComponent;
        }

        void addRotor(UStaticMesh* rotorMesh, float x, float y, float z)
        {
            addRotor(rotorMesh, x, y, z, rotorStartAngle(x,y));
        }

        virtual void setRotorRotation(uint8_t index, float angle)
        {
            _rotorMeshComponents[index]->SetRelativeRotation(FRotator(0,
                                                                      angle,
                                                                      0));
        }

}; // class Vehicle
