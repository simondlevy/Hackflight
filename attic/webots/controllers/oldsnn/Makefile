#  Copyright (C) 2024 Simon D. Levy
 
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

HFSRC_DIR = $(HOME)/Desktop/hackflight/src

# TeNNLab framework stuff ------------------------------------------

FDIR = $(HOME)/Desktop/framework

PDIR =  $(FDIR)/processors/risp

VIZ_NET = $(shell pwd)/networks/difference_risp_train.txt
VIZ_HOST = "localhost"
VIZ_PORT = 8100

# -------------------------------------------------------------------

CFLAGS = -O3 -std=c++11 -Wall -Wextra

INCLUDE = -I$(HFSRC_DIR) -I$(FDIR)/include -I$(FDIR)/include/utils
LIBRARIES = $(FDIR)/lib/libframework.a \
		    $(PDIR)/lib/libproc1.a \
		    $(PDIR)/lib/libproc2.a

NAME = oldsnn

CXX_SOURCES = main.cpp

all: $(NAME)

# Use the Webots C API in your C++ controller program
USE_C_API = true

VERBOSE = 1

run:
	webots ../../worlds/old.wbt &
	/usr/local/webots/webots-controller --stdout-redirect $(NAME)

runlog:
	webots ../../worlds/old.wbt &
	/usr/local/webots/webots-controller $(NAME) | tail +11 > log.csv

runviz:
	webots ../../worlds/old.wbt &
	/usr/local/webots/webots-controller --stdout-redirect $(NAME) $(VIZ_PORT) &
	sleep 1
	cd $(FDIR)/viz; \
 	love . -i '{"source":"request","port":$(VIZ_PORT),"host":$(VIZ_HOST)}' \
     	-n $(VIZ_NET) --show_spike_count --remove_unnecessary_neuron \
    		--set_num_screen_shot 0 > /dev/null

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

edit:
	vim main.cpp

plot:
	./plotlog.py

log:
	libreoffice log.csv
