/*
* HackflightSimEditor.Target.cs: Edtor Target script for HackflightSim
*
* Copyright (C) 2018 Simon D. Levy
*
* MIT License
*/

using UnrealBuildTool;
using System.Collections.Generic;

public class HackflightSimEditorTarget : TargetRules
{
	public HackflightSimEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
        DefaultBuildSettings = BuildSettingsVersion.V2;
		ExtraModuleNames.Add("HackflightSim");
	}
}
