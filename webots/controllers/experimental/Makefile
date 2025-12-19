#  Copyright (C) 2025 Simon D. Levy
 
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, in version 3.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program. If not, see <http:--www.gnu.org/licenses/>.

# Support different worlds and setpoints --------------------------------------

world = simple

setpoint = human

# -----------------------------------------------------------------------------

ALL = experimental

OPENCV_DIR = /usr/include/opencv4

SIMSENSORS_DIR = $(HOME)/Desktop/simsensors/src

CFLAGS = -O3 -std=c++11 -Wall -Wextra

SRCDIR = ../../../src
INCLUDE = -I$(SRCDIR) -I$(SRCDIR)/pid/slowfast -I$(OPENCV_DIR) -I$(SIMSENSORS_DIR)

LIBRARIES = -L/usr/lib/x86_64-linux-gnu/ -lopencv_highgui -lopencv_core -lopencv_imgproc

CXX_SOURCES = $(ALL).cpp

all: $(ALL)

# Use the Webots C API in your C++ controller program
USE_C_API = true

VERBOSE = 1

run:
	webots ../../worlds/$(world).wbt &
	/usr/local/webots/webots-controller --stdout-redirect $(ALL) $(world) $(setpoint) \
		../../protos


edit:
	vim $(ALL).cpp

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

