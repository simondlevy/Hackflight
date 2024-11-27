# Custom physics plugin for Hackflight simulator using Estimated Kalman Filter
# for state and standard C++ PID controllers
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

INCLUDE = -I../../../../src -I$(HOME)/Documents/Arduino/libraries/TinyEKF/src

CXX_SOURCES = ekf.cpp ../support.cpp

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
