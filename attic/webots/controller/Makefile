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

# Command-line variable; e.g. make run world=twoexit ------------------------
world = simple

EXE = $(shell basename "`pwd`")

WORLDS_DIR = ../../worlds

SIMSENSORS_DIR = $(HOME)/Desktop/simsensors/src

CFLAGS = -O3 -std=c++11 -Wall -Wextra

SRC_DIR = ../../../src
INCLUDE = -I$(SRC_DIR) -I$(SRC_DIR)/pid/slowfast -I$(SIMSENSORS_DIR)

LIBRARIES = -L/usr/lib/x86_64-linux-gnu

CXX_SOURCES = main.cpp platform_webots.cpp

FILES_TO_REMOVE += *.csv

all: $(EXE)

# Use the Webots C API in your C++ controller program
USE_C_API = true

VERBOSE = 1

# https://stackoverflow.com/a/49524393
.EXPORT_ALL_VARIABLES:
WEBOTS_WORLD = $(world)
WEBOTS_PATH = $(shell pwd)

run:
	webots $(WORLDS_DIR)/$(world).wbt &
	/usr/local/webots/webots-controller --stdout-redirect $(EXE) $(world)

edit:
	vim main.cpp


### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

