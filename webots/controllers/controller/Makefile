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

# TeNNLab framework stuff ----------------------------------------------------

FDIR = $(HOME)/Desktop/framework-open

DDIR = $(HOME)/Desktop/2025-diff-network

NET = $(DDIR)/diff_net.txt

VIZ_PORT = 8100

# -----------------------------------------------------------------------------

CFLAGS = -O3 -std=c++11 -Wall -Wextra

SRCDIR = ../../../src
INCLUDE = -I$(SRCDIR)

NAME = controller

CXX_SOURCES = controller.cpp

all: $(NAME)

# Use the Webots C API in your C++ controller program
USE_C_API = true

VERBOSE = 1

runstandard:
	cp ../../plugins/physics/standard/libstandard.so \
		../../plugins/physics/custom/libcustom.so
	webots ../../worlds/simple.wbt &
	/usr/local/webots/webots-controller --stdout-redirect $(NAME)

runhaskell:
	cp ../../plugins/physics/haskell/libhaskell.so \
		../../plugins/physics/custom/libcustom.so
	webots ../../worlds/simple.wbt &
	/usr/local/webots/webots-controller --stdout-redirect $(NAME)

runsnn:
	sh $(DDIR)/scripts/diff_network.sh 1000 $(FDIR) > $(NET)
	cp ../../plugins/physics/snn/libsnn.so \
		../../plugins/physics/custom/libcustom.so
	$(HOME)/Desktop/spikeplotter/spikeplot.py  --ids 'x,y,z' --address localhost --port $(VIZ_PORT) --display-counts &
	webots ../../worlds/simple.wbt &
	/usr/local/webots/webots-controller --stdout-redirect $(NAME)

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

edit:
	vim main.cpp

