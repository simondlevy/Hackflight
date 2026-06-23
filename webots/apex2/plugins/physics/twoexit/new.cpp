


























DLLEXPORT void webots_physics_step() 
{
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
