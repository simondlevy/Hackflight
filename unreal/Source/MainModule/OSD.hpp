/*
* On-Screen Display for MulticopterSim
*
* Copyright (C) 2018 Simon D. Levy
*
* MIT License
*/

#pragma once

#include "Engine.h"

static float _min(float a, float b)
{
    return a < b ? a : b;
}

static void osd(char * buf, bool err=false, bool overwrite=false)
{
    if (GEngine && GEngine->GameViewport) {

        // Show message in yellow, error in red
        FColor TEXT_COLOR = err ? FColor::Red : FColor::Yellow;

        // Get viewport size for scaling text
        FVector2D ViewportSize = FVector2D(1, 1);
        GEngine->GameViewport->GetViewportSize(ViewportSize);

        // Scale text to fit message to screen
        float  textScale = _min(ViewportSize.X / 6 / strlen(buf), 2);

        // -1 = no overwrite (0 for overwrite); 5.f = arbitrary time to display; true = newer on top
        GEngine->AddOnScreenDebugMessage(overwrite ? 0 : -1, 5.f, TEXT_COLOR, FString(buf), true, FVector2D(textScale,textScale));
    }
}
