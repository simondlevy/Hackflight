# Adapted from hello_world.py example: https://docs.openmv.io/openmvcam/tutorial/script_structure.html

import sensor, image, time
from pyb import UART
import msppg

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)  # Set frame size to QVGA (160x120)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

# Invert the image
sensor.__write_reg(0x0C, sensor.__read_reg(0x0C) | (1 << 7))

# Start serial comms on UART1
uart = UART(1, 115200)

while(True):
    clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()         # Take a picture and return the image.
    print(clock.fps())              # Note: OpenMV Cam runs about half as fast when connected
                                    # to the IDE. The FPS should increase once disconnected.
    uart.write('hello\n')
