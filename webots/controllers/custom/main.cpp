
#include <webots/camera.h>
#include <webots/emitter.h>
#include <webots/keyboard.h>
#include <webots/robot.h>

#include "js.h"

#define TIMESTEP 32  // ms
#define KEY_INCREMENT 0.05

int main() {
  double command[3] = {0.0, 0.0, 0.0};
  WbDeviceTag gEmitter = 0;

  wb_robot_init();
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIMESTEP * 2);
  // Handle joystick
  jsJoystick *gJoystick = new jsJoystick();
  if (gJoystick->notWorking()) {
    delete gJoystick, gJoystick = NULL;
    printf("No joystick available, using keyboard control (click in the 3D windows before pressing keys).\n");
    printf("Available control keys: up, down, right, left, page up, page down and space (reset).\n");
    wb_keyboard_enable(TIMESTEP);
  }

  // Handle emitter
  gEmitter = wb_robot_get_device("emitter");
  if (!gEmitter)
    printf("!!! joystick :: reset :: emitter is not available.\n");

  while (wb_robot_step(TIMESTEP) != -1) {

    // Send joystick value.
    if (gEmitter) {
      // read joystick.
      if (gJoystick) {
        float axes[12];
        int buttons[12];
        gJoystick->read(buttons, axes);
        command[0] = (double)-axes[1];
        command[1] = (double)axes[3];
        command[2] = (double)axes[2];
      } else {
        switch (wb_keyboard_get_key()) {
          case WB_KEYBOARD_DOWN:
            command[0] -= KEY_INCREMENT;
            break;
          case WB_KEYBOARD_UP:
            command[0] += KEY_INCREMENT;
            break;
          case WB_KEYBOARD_LEFT:
            command[1] += KEY_INCREMENT;
            break;
          case WB_KEYBOARD_RIGHT:
            command[1] -= KEY_INCREMENT;
            break;
          case WB_KEYBOARD_PAGEUP:
            command[2] += KEY_INCREMENT;
            break;
          case WB_KEYBOARD_PAGEDOWN:
            command[2] -= KEY_INCREMENT;
            break;
          case ' ':  // space -> reset
            command[0] = 0.0;
            command[1] = 0.0;
            command[2] = 0.0;
        }
      }
      // setup emitter buffer
      if (command[0] || command[1] || command[2])
        printf("command = ( %g , %g , %g )\n", command[0], command[1], command[2]);
      wb_emitter_send(gEmitter, command, sizeof(command));
    }
  }

  wb_robot_cleanup();
  return 0;
}
