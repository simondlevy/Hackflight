# Custom physics plugin for Hackflight simulator 
#
#  Copyright (C) 2025 Simon D. Levy
#
#  This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
#  This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http:--www.gnu.org/licenses/>.

ALL = $(shell basename "`pwd`")

OPENCV_DIR = /usr/include/opencv4

SIMSENSORS_DIR = $(HOME)/Desktop

CFLAGS = -O3 -std=c++11 -Wall -Wextra

SRC_DIR = ../../../../src

INCLUDE = -I$(SRC_DIR) -I$(SRC_DIR)/newpids -I$(SIMSENSORS_DIR) -I$(OPENCV_DIR)

CXX_SOURCES = $(ALL).cpp

LIBRARIES = -L/usr/lib/x86_64-linux-gnu/ \
			-lopencv_highgui -lopencv_core -lopencv_imgproc

all: lib$(ALL).so

edit:
	vim $(ALL).cpp

FILES_TO_REMOVE += copilot*

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
