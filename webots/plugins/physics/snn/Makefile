# Custom physics plugin for Hackflight simulator using TeNNLab Spiking Neural
# networks for control
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

CFLAGS = -O3 -std=c++11 -Wall -Wextra

INCLUDE = -I../../../simsrc -I../../../../src

CXX_SOURCES = ../support.cpp

# TeNNLab framework stuff ----------------------------------------------------

ROOT = $(HOME)/Desktop

FDIR = $(HOME)/Desktop/framework

PDIR =  $(FDIR)/processors/risp

INCLUDE += -I$(ROOT) -I$(FDIR)/include -I$(FDIR)/include/utils/levy -I$(FDIR)/cpp-apps/include
CXX_SOURCES += $(FDIR)/src/io_stream.cpp
CXX_SOURCES += $(FDIR)/src/jspace.cpp
CXX_SOURCES += $(FDIR)/src/network.cpp
CXX_SOURCES += $(FDIR)/src/properties.cpp
CXX_SOURCES += $(FDIR)/src/spike_encoding.cpp

INCLUDE += -I$(PDIR)/include
CXX_SOURCES += $(PDIR)/src/processor.cpp
CXX_SOURCES += $(PDIR)/src/static_proc.cpp

VIZ_NET = $(shell pwd)/networks/difference_risp_train.txt
VIZ_HOST = "localhost"
VIZ_PORT = 8100

# ----------------------------------------------------------------------------

all: libsnn.so

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
