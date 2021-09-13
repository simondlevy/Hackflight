/*
 * Abstract camera class for MulticopterSim
 *
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include "Utils.hpp"

class Camera {

    friend class Vehicle;

    public:

        // Default position w.r.t vehicle
        static constexpr float X = 0.2;
        static constexpr float Y = 0.0;
        static constexpr float Z = 0.3;

        // Arbitrary array limits supporting statically declared assets
        static const uint8_t MAX_CAMERAS = 10; 

        // Supported resolutions
        typedef enum {

            RES_640x480,
            RES_1280x720,
            RES_1920x1080,
            RES_COUNT

        } Resolution_t;

    private:

        float _x = 0;
        float _y = 0;
        float _z = 0;

        Resolution_t _res;

        // Byte array for RGBA image
        uint8_t * _imageBytes = NULL;

    protected:

        // Image size and field of view, set in constructor
        uint16_t _rows = 0;
        uint16_t _cols = 0;

        // Initial FOV can be overridden by setFov()
        float    _fov  = 0;

        // UE4 resources, set in Vehicle::addCamera()
        USceneCaptureComponent2D * _captureComponent = NULL;
        FRenderTarget            * _renderTarget = NULL;

        Camera(float fov, Resolution_t resolution, float x=Camera::X, float y=Camera::Y, float z=Camera::Z)
        {
            uint16_t rowss[3] = {480, 720, 1080};
            uint16_t colss[3] = {640, 1280, 1920};

            _rows = rowss[resolution];
            _cols = colss[resolution];
            _res  = resolution;
            _fov = fov;

            // Set position w.r.t. vehicle
            _x = x;
            _y = y;
            _z = z;

            // Create a byte array sufficient to hold the RGBA image
            _imageBytes = new uint8_t [_rows*_cols*4]();

            // These will be set in Vehicle::addCamera()
            _captureComponent = NULL;
            _renderTarget = NULL;
        }

        // Called by Vehicle::addCamera()
        virtual void addToVehicle(APawn * pawn, USpringArmComponent * springArm, uint8_t id)
        {
            // Get static assets for all render targets.  This provides less
            // flexibility than creating it dynamically, but acquiring the
            // pixels seems to run twice as fast.
            static ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>
                cameraTextureObjects[RES_COUNT][MAX_CAMERAS] =
            { 
                {
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_640x480_1")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_640x480_2")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_640x480_3")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_640x480_4")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_640x480_5")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_640x480_6")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_640x480_7")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_640x480_8")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_640x480_9")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_640x480_10")) 
                },
                {
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1280x720_1")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1280x720_2")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1280x720_3")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1280x720_4")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1280x720_5")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1280x720_6")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1280x720_7")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1280x720_8")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1280x720_9")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1280x720_10")) 
                },
                {
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1920x1080_1")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1920x1080_2")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1920x1080_3")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1920x1080_4")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1920x1080_5")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1920x1080_6")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1920x1080_7")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1920x1080_8")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1920x1080_9")),
                    ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D>(TEXT("/Game/MulticopterSim/RenderTargets/renderTarget_1920x1080_10")) 
                }
            };

            UTextureRenderTarget2D * textureRenderTarget2D = cameraTextureObjects[_res][id].Object;

            // Create a scene-capture component and set its target to the render target
            _captureComponent = pawn->CreateDefaultSubobject<USceneCaptureComponent2D >(makeName("Capture", id));
            _captureComponent->SetWorldScale3D(FVector(0.1,0.1,0.1));
            _captureComponent->SetupAttachment(springArm, USpringArmComponent::SocketName);
            _captureComponent->SetRelativeLocation(100*FVector(_x, _y, _z));  // m => cm
            _captureComponent->TextureTarget = textureRenderTarget2D;

            // Get the render target resource for copying the image pixels
            _renderTarget = textureRenderTarget2D->GameThread_GetRenderTargetResource();
            
            // Set the initial FOV
            setFov(_fov);
        }

        // Override this method for your video application
        virtual void processImageBytes(uint8_t * bytes) { (void)bytes; }

        // Sets current FOV
        void setFov(float fov)
        {
            _captureComponent->FOVAngle = fov;
        }

    public:

        // Called on main thread
        void grabImage(void)
        {
            // Read the pixels from the RenderTarget
            TArray<FColor> renderTargetPixels;
            _renderTarget->ReadPixels(renderTargetPixels);

            // Copy the RBGA pixels to the private image
            FMemory::Memcpy(_imageBytes, renderTargetPixels.GetData(), _rows*_cols*4);

            // Virtual method implemented in subclass
            processImageBytes(_imageBytes);
        }

        virtual ~Camera()
        {
            delete _imageBytes;
        }

}; // Class Camera
