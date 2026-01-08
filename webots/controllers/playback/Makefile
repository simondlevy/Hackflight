#  Copyright (C) 2026 Simon D. Levy
 
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

# Command-line variables; e.g. make run world=maze ---------------------------
world = simple
logfile = ../controller/log.csv
# ----------------------------------------------------------------------------

EXE = $(shell basename "`pwd`")

OPENCV_DIR = /usr/include/opencv4

SIMSENSORS_DIR = $(HOME)/Desktop

CFLAGS = -O3 -std=c++11 -Wall -Wextra

SRC_DIR = ../../../src

INCLUDE = -I$(SRC_DIR) -I$(SRC_DIR)/pid/slowfast -I$(SIMSENSORS_DIR) \
		  -I$(OPENCV_DIR)

LIBRARIES = -L/usr/lib/x86_64-linux-gnu \
			-lopencv_highgui -lopencv_core -lopencv_imgproc

CXX_SOURCES = main.cpp

FILES_TO_REMOVE += *.csv

all: $(EXE)

# Use the Webots C API in your C++ controller program
USE_C_API = true

VERBOSE = 1

run:
	webots ../../worlds/$(world).wbt &
	/usr/local/webots/webots-controller --stdout-redirect $(EXE) $(world) \
		$(logfile)

edit:
	vim main.cpp

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

