/*
 * Build script for HackflightModule, including OpenCV support
 *
 * Copyright (C) 2018 Simon D. Levy
 *
 * MIT License
 */


using UnrealBuildTool;
using System;
using System.IO;

public class HackflightModule : ModuleRules
{
    // Change this to agree with your Arduino libraries install location
    // private static string ARDUINO_PATH = "C:\\Users\\Administrator\\Documents\\Arduino\\libraries\\";
    private static string ARDUINO_PATH = "C:\\Users\\nguyennq23\\Documents\\Arduino\\libraries\\";

    public HackflightModule(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new string[] 
                { "Core", "CoreUObject", "Engine", "InputCore" });

        PrivateDependencyModuleNames.AddRange(new string[] { "MainModule" });

        // Hackflight support --------------------------------------------------------------------

        PrivateIncludePaths.Add(ARDUINO_PATH + "Hackflight\\src");
        PrivateIncludePaths.Add(ARDUINO_PATH + "RoboFirmwareToolkit\\src");
        // OpenCV support --------------------------------------------------------------------
    
        // Create OpenCV Path
        string OpenCVPath = Path.Combine(ThirdPartyPath, "OpenCV");

        // Get Library Path
        string LibPath = Path.Combine(OpenCVPath, "lib");

        //Add Include path
        PublicIncludePaths.AddRange(new string[] { Path.Combine(OpenCVPath, "include") });

        // Add Library Path
        PublicLibraryPaths.Add(LibPath);

        // Add Libraries
        PublicAdditionalLibraries.Add("opencv_world452.lib");
        PublicDelayLoadDLLs.Add("opencv_world452.dll");

        PublicDefinitions.Add(string.Format("WITH_OPENCV_BINDING=1"));
        PublicDefinitions.Add("_USE_OPENCV");
    }

    private string ThirdPartyPath
    {
        get { return Path.GetFullPath(Path.Combine(ModuleDirectory, "ThirdParty")); }
    }
}

