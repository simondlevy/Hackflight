This directory contains a C++ library that allows you to use various controllers for writing flight simulators
on Windows, Linus, and OS X:

* Taranis DX9
* XBox 360 controller
* PS3 controller
* Logitech Extreme Pro 3D joystick

``` controller_t controller; // TARANIS, PS3, XBOX360, etc.

    controller = controllerInit();

    controllerRead(controller, demands);

    controllerClose();
```

The <b>controllerInit()</b> method automatically figures out what kind of controller you have, and returns
a value (<b>TARANIS</b>, <b>PS3</b>, etc.) that you can use to treat different controllers differently if you
need to.
