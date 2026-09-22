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

EMBEDDER = $(HOME)/Desktop/embedded-framework/bin/framework_embedder

DPSRC = $(HOME)/Desktop/dronepong/embedded_dronepong.cpp

FPGA_DIR = $(HOME)//Desktop/fpga
FPGA_TL_DIR = $(FPGA_DIR)/tennlab_fpga

NET = $(fr)/cpp-apps/applications/dronepong/networks/dronepong_risp_train.txt 

CFLAGS = -O3 -std=c++17 -Wall -Wextra

SRC_DIR = ../../../../../src

INCLUDE = -I../../../../ -I$(SRC_DIR) -I$(SRC_DIR)/control \
		  -I$(SIMSENSORS_DIR) -I$(OPENCV_DIR) -I$(HOME)/Desktop/dronepong \
		  -I$(FPGA_TL_DIR)/src 

CXX_SOURCES = fpga.cpp  $(DPSRC) $(FPGA_TL_DIR)/posix/uart.cpp

LIBRARIES = -L/usr/lib/x86_64-linux-gnu/ \
			-lopencv_highgui -lopencv_core -lopencv_imgproc

all: lib$(NAME).so

load: $(NET)
	$(FPGA_DIR)/utils/load.py -t cmoda7_35t $(NET)
	$(FPGA_TL_DIR)/utils/compile.py -d $(NET)

edit:
	vim $(NAME).cpp

fclean:
	rm -rf ~/.cache/neuro_fpga/

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
