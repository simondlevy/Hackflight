# Custom physics plugin for Hackflight simulator using NASA Copilit Haskell
# compiler
#
#  Copyright (C) 2024 Simon D. Levy
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

INCLUDE = -I../../../../src

HASKDIR = ../../../../haskell

CORE_TASK = ../../../../src/sim/controllers/haskell/CoreTask.hs

CXX_SOURCES = haskell.cpp copilot_core.cpp ../support.cpp

FILES_TO_REMOVE += copilot*

all: libhaskell.so

copilot_core.cpp : $(CORE_TASK) $(HASKDIR)/*.hs $(HASKDIR)/pids/*.hs
	runhaskell -i$(HASKDIR) -i$(HASKDIR)/pids $(CORE_TASK)
	mv copilot_core.c copilot_core.cpp

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
