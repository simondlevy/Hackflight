// Returns false on collision, true otherwise
// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    static int _rangefinder_distances_mm[8];

    const auto message = PluginHelper::GetMessage();

    // Replace open-loop setpoint with setpoint from autopilot if
    // available
    const auto setpoint = message.mode == hf::kModeAutonomous ?
        getSetpoint(_rangefinder_distances_mm) : message.setpoint;

    // Get vehicle pose based on setpoint
    const auto pose = ahelper_->GetPose(message.mode, setpoint);

    auto rangefinder = ahelper_->robot.rangefinders["VL53L5-forward"];

    // Grab rangefinder distances for next iteration
    rangefinder.read(pose, ahelper_->world, _rangefinder_distances_mm);

    // Log data to file
    ahelper_->WriteToLog(pose, _rangefinder_distances_mm, 8);

    // Display rangefinder distances
    simsens::RangefinderVisualizer::show(
            _rangefinder_distances_mm,
            rangefinder.min_distance_m,
            rangefinder.max_distance_m,
            8, 1, kRangefinderDisplayScaleup);
}

DLLEXPORT void webots_physics_cleanup() 
{
    delete ahelper_;
}

DLLEXPORT void webots_physics_init() 
{
    ahelper_ = new AutopilotHelper("twoexit");
}
