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

NAME = $(shell basename "`pwd`")

OPENCV_DIR = /usr/include/opencv4
SIMSENSORS_DIR = $(HOME)/Desktop

EMBEDDER_DIR = $(HOME)/Desktop/framework-embedder
FPGA_DIR = $(HOME)//Desktop/fpga/tennlab_fpga

NET = $(EMBEDDER_DIR)/networks/dronepong_risp_train.txt 

CPP = embedded_dronepong.cpp
HPP = dronepong_fpga.hpp

CFLAGS = -O3 -std=c++17 -Wall -Wextra

SRC_DIR = ../../../../../src

INCLUDE = -I../../../../ -I$(SRC_DIR) -I$(SRC_DIR)/control \
		  -I$(FPGA_DIR)/src -I$(SIMSENSORS_DIR) -I$(OPENCV_DIR)

CXX_SOURCES = fpga.cpp $(CPP) $(FPGA_DIR)/posix/uart.cpp

LIBRARIES = -L/usr/lib/x86_64-linux-gnu/ \
			-lopencv_highgui -lopencv_core -lopencv_imgproc

FILES_TO_REMOVE += $(CPP) $(HPP)

all: lib$(NAME).so

embedded_dronepong.cpp: $(NET) $(HPP)
	$(EMBEDDER_DIR)/bin/framework_embedder < $(NET) > embedded_dronepong.cpp

$(HPP): $(NET)
	$(FPGA_DIR)/utils/mkhdr.py $(NET) > $(HPP)

edit:
	vim $(NAME).cpp

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
