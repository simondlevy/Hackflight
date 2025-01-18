# 
#  Run TeNNLab spiking neural network
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

CFLAGS = -O3 -std=c++11 -Wall -Wextra

CXX_SOURCES = snn.cpp  

# TeNNLab framework stuff ----------------------------------------------------

FDIR = $(HOME)/Desktop/framework

PDIR =  $(FDIR)/processors/risp

INCLUDE += -I$(FDIR)/include -I$(FDIR)/include/utils -I$(FDIR)/cpp-apps/include
CXX_SOURCES += $(FDIR)/src/io_stream.cpp
CXX_SOURCES += $(FDIR)/src/jspace.cpp
CXX_SOURCES += $(FDIR)/src/network.cpp
CXX_SOURCES += $(FDIR)/src/properties.cpp
CXX_SOURCES += $(FDIR)/src/spike_encoding.cpp

INCLUDE += -I$(PDIR)/include
CXX_SOURCES += $(PDIR)/src/processor.cpp
CXX_SOURCES += $(PDIR)/src/static_proc.cpp

# ----------------------------------------------------------------------------

all: snn

snn: main.o
	g++ -o snn main.o

main.o: main.cpp
	g++ $(CFLAGS) -c main.cpp

edit:
	vim main.cpp

clean:
	rm -f *.o snn
