/*
 * Build script for CopilotModule
 *
 * Copyright (C) 2021 Simon D. Levy
 *
 * MIT License
 */


using UnrealBuildTool;
using System;
using System.IO;

public class CopilotModule : ModuleRules
{
    public CopilotModule(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new string[] 
                { "Core", "CoreUObject", "Engine", "InputCore", "Landscape" });

        PrivateDependencyModuleNames.AddRange(new string[] { "MainModule" });
    }
}
