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

SRC = $(HOME)/Desktop/hackflight/src
OPENCV_INC = /usr/include/opencv4
OPENCV_LIB = -L/usr/lib/x86_64-linux-gnu/ -lopencv_highgui -lopencv_core -lopencv_imgproc

CFLAGS = -O3 -std=c++11 -Wall -Wextra

world = ../../../worlds/maze.wbt
robot = ../../../protos/DiyQuad.proto

all: main

run: main
	./main $(world) $(robot)

main: main.o
	g++ -o main main.o $(OPENCV_LIB)

main.o: main.cpp  *.h*
	g++ -c -I$(SRC) -I$(OPENCV_INC) $(CFLAGS) main.cpp

clean:
	rm -f main

edit:
	vim main.cpp

