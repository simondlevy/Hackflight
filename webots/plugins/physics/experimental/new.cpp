static bool run_normal()
{
    // Get sim info from main program
    int bytes_received = 0;
    siminfo_t siminfo = {};
    const auto buffer = (siminfo_t *)dWebotsReceive(&bytes_received);
    if (bytes_received == sizeof(siminfo_t)) {
        memcpy(&siminfo, buffer, sizeof(siminfo));
    }

    // This happens at startup
    if (siminfo.framerate == 0) {
        return true;
    }

    static simsens::SimRangefinder * _simRangefinder;
    static simsens::RangefinderVisualizer * _rangefinderVisualizer;
    static simsens::RobotParser _robotParser;
    static simsens::WorldParser _worldParser;

    // Load world and robot info first time around
    if (!_simRangefinder) {

        char path[1000];

        sprintf(path, "%s/../../worlds/%s.wbt", siminfo.path, siminfo.worldname);
        _worldParser.parse(path);

        sprintf(path, "%s/../../protos/DiyQuad.proto", siminfo.path);
        _robotParser.parse(path);

        _simRangefinder = _robotParser.rangefinders[0];

        _rangefinderVisualizer = new simsens::RangefinderVisualizer(_simRangefinder);
    }

    // Update to get the current pose
    const SimInnerLoop::pose_t pose = _innerLoop.step(siminfo);

    // Set robot posed based on state and starting position, negating for
    // rightward negative
    const double robot_x = siminfo.start_x + pose.x;
    const double robot_y = siminfo.start_y - pose.y;
    const double robot_z = siminfo.start_z + pose.z;

    // Stop if we detected a collision
    const bool debug = true;
    if (simsens::CollisionDetector::detect(
                simsens::vec3_t{robot_x, robot_y, robot_x},
                _worldParser.walls, debug)) {
        return false;
    }

    // Get simulated rangefinder distances
    int rangefinder_distances_mm[1000] = {}; // arbitrary max size
    int rangefinder_width=0, rangefinder_height=0;
    simsens::vec3_t dbg_intersection = {};
    _simRangefinder->read(
            simsens::pose_t{robot_x, robot_y, robot_z,
            pose.phi, pose.theta, pose.psi},
            _worldParser.walls,
            rangefinder_distances_mm,
            rangefinder_width,
            rangefinder_height);

    for (int k=0; k<rangefinder_width; ++k) {
        fprintf(_logfp, "%d%c", rangefinder_distances_mm[k],
                (k==rangefinder_width-1)?'\n':',');
    }
    fflush(_logfp);

    _rangefinderVisualizer->show(rangefinder_distances_mm, RANGEFINDER_DISPLAY_SCALEUP);

    // Turn Euler angles into quaternion, negating psi for nose-right positive 
    const axis3_t euler = {pose.phi, pose.theta, -pose.psi};
    axis4_t quat = {};
    Num::euler2quat(euler, quat);

    const dQuaternion q = {quat.w, quat.x, quat.y, quat.z};
    dBodySetQuaternion(_robot, q);

    dBodySetPosition(_robot, robot_x, robot_y, robot_z);

    if (_red_ball) {
        dBodySetPosition(_red_ball,
                dbg_intersection.x, dbg_intersection.y, dbg_intersection.z);
    }

    return true;
}

}

// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    if (_robot == NULL) {
        return;
    }

}
