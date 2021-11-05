/*
* HackflightSim.Target.cs: Target script for HackflightSim
*
* Copyright (C) 2018 Simon D. Levy
*
* MIT License
*/

using UnrealBuildTool;
using System.Collections.Generic;

public class HackflightSimTarget : TargetRules
{
	public HackflightSimTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V2;
		ExtraModuleNames.Add("HackflightSim");
	}
}
