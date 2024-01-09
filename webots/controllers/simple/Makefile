CFLAGS = -Wno-unused-variable

SRC = ../../../src

# INCLUDE = -I$(HOME)/Desktop/simdvs/cpp -I$(SRC)
INCLUDE = -I$(SRC)

CXX_SOURCES = simple.cpp console.cpp

#ifeq ($(OS),Windows_NT)
#	OPENCV = "D:/opencv-mingw"
#	INCLUDE = -I$(OPENCV)/include
#	LIBRARIES = -L$(OPENCV)/x64/mingw/bin 
#	LIBRARIES += -lopencv_core455 -lopencv_videoio455 -lopencv_highgui455
				
#else
#	INCLUDE += -I/usr/include/opencv4 -I../../joystick/linux
#	LIBRARIES = -L/usr/lib/x86_64-linux-gnu
#    LIBRARIES += -lopencv_core -lopencv_videoio -lopencv_highgui 
#endif

# Use the Webots C API in your C++ controller program
USE_C_API = true

VERBOSE = 1

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
