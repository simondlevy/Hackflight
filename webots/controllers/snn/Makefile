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

# TeNNLab framework stuff ------------------------------------------

FDIR = $(HOME)/Desktop/framework

NETWORK = $(shell pwd)/networks/new_risp.txt

CFLAGS = -O3 -std=c++11 -Wall -Wextra

PDIR =  $(FDIR)/processors/risp

VIZ_HOST = "localhost"
CLIMBRATE_VIZ_PORT = 8100
YAWRATE_VIZ_PORT = 8200

# -------------------------------------------------------------------

CFLAGS = -O3 -std=c++11 -Wall -Wextra

SRCDIR = ../../../src

INCLUDE = -I$(SRCDIR) -I$(FDIR)/include -I$(FDIR)/include/utils
LIBRARIES = $(FDIR)/lib/libframework.a \
		    $(PDIR)/lib/libproc1.a \
		    $(PDIR)/lib/libproc2.a

NAME = snn

CXX_SOURCES = $(NAME).cpp

all: $(NAME)

# Use the Webots C API in your C++ controller program
USE_C_API = true

VERBOSE = 1

run:
	webots ../../worlds/simple.wbt &
	/usr/local/webots/webots-controller $(NAME) $(NETWORK) | tail +9 > log.csv &

bigrun:
	webots ../../worlds/simple.wbt &
	/usr/local/webots/webots-controller $(NAME) $(NETWORK) | tail +11 > log.csv &
	sleep 1
	cd $(FDIR)/viz; \
		love . -i '{"source":"request","port":$(CLIMBRATE_VIZ_PORT),"host":$(VIZ_HOST)}' \
		-n $(NETWORK) --show_spike_count --remove_unnecessary_neuron \
		--set_num_screen_shot 0 > /dev/null &
	cd $(FDIR)/viz; \
		love . -i '{"source":"request","port":$(YAWRATE_VIZ_PORT),"host":$(VIZ_HOST)}' \
		-n $(NETWORK) --show_spike_count --remove_unnecessary_neuron \
		--set_num_screen_shot 0 > /dev/null &


show:
	@$(FDIR)/cpp-apps/bin/function_risp -a test -n $(NETWORK) \
		--show_observations --show_actions | $(FDIR)/cpp-apps/pyplot/diffplot.py

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

edit:
	vim $(NAME).cpp

log:
	libreoffice log.csv


