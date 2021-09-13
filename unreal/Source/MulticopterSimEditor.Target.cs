/*
* MulticopterSimEditor.Target.cs: Edtor Target script for MulticopterSim
*
* Copyright (C) 2018 Simon D. Levy
*
* MIT License
*/

using UnrealBuildTool;
using System.Collections.Generic;

public class MulticopterSimEditorTarget : TargetRules
{
	public MulticopterSimEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;

        DefaultBuildSettings = BuildSettingsVersion.V2;

		ExtraModuleNames.AddRange( new string[] { 
                "MainModule", 
                "CopilotModule"
                } );
	}
}
