#
# Makefile for Linux joystick example
#
# Copyright (C) 2019 Simon D. Levy
# 
# MIT License
# 

ALL = joytest 

JOYDIR = ../../../Source/MainModule/joystick

all: $(ALL)

CFLAGS = -Wall -std=c++11

all: $(ALL)

test: joytest
	./joytest

joytest: joytest.o JoystickLinux.o
	g++ -o joytest joytest.o JoystickLinux.o

joytest.o: ../joytest.cpp 
	g++ -c $(CFLAGS) -I$(JOYDIR) ../joytest.cpp 

JoystickLinux.o: $(JOYDIR)/JoystickLinux.cpp $(JOYDIR)/Joystick.h
	g++ -c $(CFLAGS) -I$(JOYDIR) $(JOYDIR)/JoystickLinux.cpp

clean:
	rm -rf $(ALL) *.o *~

edit:
	vim ../joytest.cpp
