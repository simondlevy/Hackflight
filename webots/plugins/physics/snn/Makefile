# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Webots Makefile system 
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include

HFSRC_DIR = $(HOME)/Desktop/hackflight/src

# TeNNLab framework stuff ------------------------------------------

FDIR = $(HOME)/Desktop/framework

PDIR =  $(FDIR)/processors/risp

VIZ_NET = $(shell pwd)/networks/difference_risp_train.txt
VIZ_HOST = "localhost"
VIZ_PORT = 8100

# -------------------------------------------------------------------

CFLAGS = -O3 -std=c++11 -Wall -Wextra

INCLUDE = -I$(HFSRC_DIR) -I$(FDIR)/include -I$(FDIR)/include/utils \
		  -I$(FDIR)/cpp-apps/include

CXX_SOURCES = main.cpp 

include $(FDIR)/webots-makefile.include

all: libsnn.so

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
