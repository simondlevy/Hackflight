// We currently support these controllers
enum Controller { NONE, TARANIS, SPEKTRUM, EXTREME3D, PS3 };
static Controller controller;

// Downscaling for hypersensitive PS3 controller
static const int PS3_DOWNSCALE = 2;

