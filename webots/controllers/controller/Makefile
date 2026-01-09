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

include ../../../snn_path.makefile

NET = $(EMBDIR)/examples/difference/difference.txt

VIZ_DIR = $(HOME)/Desktop/framework/viz
VIZ_PORT = 8100

# -----------------------------------------------------------------------------

CFLAGS = -O3 -std=c++11 -Wall -Wextra

SRCDIR = ../../../src
INCLUDE = -I$(SRCDIR)

NAME = controller

CXX_SOURCES = main.cpp

all: $(NAME)

# Use the Webots C API in your C++ controller program
USE_C_API = true

VERBOSE = 1

runstandard:
	cp ../../plugins/physics/standard/libstandard.so \
		../../plugins/physics/__custom__/lib__custom__.so
	webots ../../worlds/simple.wbt &
	/usr/local/webots/webots-controller --stdout-redirect $(NAME)

runhaskell:
	cp ../../plugins/physics/haskell/libhaskell.so \
		../../plugins/physics/__custom__/lib__custom__.so
	webots ../../worlds/simple.wbt &
	/usr/local/webots/webots-controller --stdout-redirect $(NAME)

runsnn:
	cp ../../plugins/physics/snn/libsnn.so \
		../../plugins/physics/__custom__/lib__custom__.so
	webots ../../worlds/simple.wbt &
	/usr/local/webots/webots-controller --stdout-redirect $(NAME) &
	sleep 1
	cd $(VIZ_DIR); \
 	love . -i '{"source":"request","port":$(VIZ_PORT),"host":"localhost"}' \
     	-n $(NET) --set_num_screen_shot 0 --use_name_neuron \
		'{"0":"I1","1":"I2","2":"S","3":"D1","4":"D2","5":"O","6":"S2"}' \
		--set_font_size 16 \
		> /dev/null

runplank:
	cp ../../plugins/physics/plank/libplank.so \
		../../plugins/physics/__custom__/lib__custom__.so
	webots ../../worlds/simple.wbt &
	/usr/local/webots/webots-controller --stdout-redirect $(NAME)

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

edit:
	vim main.cpp

