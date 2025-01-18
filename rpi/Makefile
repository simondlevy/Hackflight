# 
#  Copyright (C) 2025 Simon D. Levy
#
# This program is free software: you can redistribute it and/or modify
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

INCLUDE += -I$(FDIR)/include -I$(FDIR)/include/utils -I$(FDIR)/cpp-apps/include \
    -I$(PDIR)/include

FSRC = $(FDIR)/src
PSRC = $(PDIR)/src

# ----------------------------------------------------------------------------

all: snn

snn: main.o io_stream.o jspace.o network.o properties.o spike_encoding.o \
	processor.o static_proc.o
	g++ -o snn main.o

main.o: main.cpp
	g++ $(CFLAGS) $(INCLUDE) -c main.cpp

io_stream.o: $(FSRC)/io_stream.cpp
	g++ $(CFLAGS) $(INCLUDE) -c $(FSRC)/io_stream.cpp

jspace.o: $(FSRC)/jspace.cpp
	g++ $(CFLAGS) $(INCLUDE) -c $(FSRC)/jspace.cpp

network.o: $(FSRC)/network.cpp
	g++ $(CFLAGS) $(INCLUDE) -c $(FSRC)/network.cpp

properties.o: $(FSRC)/properties.cpp
	g++ $(CFLAGS) $(INCLUDE) -c $(FSRC)/properties.cpp

spike_encoding.o: $(FSRC)/spike_encoding.cpp
	g++ $(CFLAGS) $(INCLUDE) -c $(FSRC)/spike_encoding.cpp

processor.o: $(PSRC)/processor.cpp
	g++ $(CFLAGS) $(INCLUDE) -c $(PSRC)/processor.cpp

static_proc.o: $(PSRC)/static_proc.cpp
	g++ $(CFLAGS) $(INCLUDE) -c $(PSRC)/static_proc.cpp

edit:
	vim main.cpp

clean:
	rm -f *.o snn
