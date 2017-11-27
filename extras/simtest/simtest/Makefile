# Makefile for simulation tester

# This file is part of Hackflight.

# Hackflight is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# Hackflight is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.

all: simtest

simtest: simtest.cpp ../../src/*.hpp ../../src/boards/sim.hpp ../../src/receivers/sim.hpp
	g++ -std=c++11 -Wall -I../../src/ -o simtest simtest.cpp

run: simtest
	./simtest

clean:
	rm -rf simtest *~ *.o
