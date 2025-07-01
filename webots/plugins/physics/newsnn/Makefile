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

SRC = ../../../../src

ROOT = $(HOME)/Desktop

CFLAGS = -O3 -std=c++11 -Wall -Wextra

INCLUDE = -I$(SRC)

CXX_SOURCES = ../support.cpp

INCLUDE = -I../../../simsrc -I../../../../src -I$(ROOT)

# TeNNLab embedded stuff ------------------------------ -----------------------

EMB = $(HOME)/Desktop/embedded-neuromorphic
NETS = $(HOME)/Desktop/tennlab-networks
RISP = $(EMB)/processors/risp

INCLUDE += -I$(RISP)/include

CXX_SOURCES += $(RISP)/src/network.cpp
CXX_SOURCES += $(NETS)/difference_risp_plank_100.cpp

# -----------------------------------------------------------------------------

all: libnewsnn.so

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
