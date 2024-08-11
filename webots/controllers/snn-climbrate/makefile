# TeNNLab framework stuff ------------------------------------------

FDIR = $(HOME)/Desktop/framework

CFLAGS = -O3 -std=c++11 -Wall -Wextra

PDIR =  $(FDIR)/processors/risp

# -------------------------------------------------------------------

CFLAGS = -O3 -std=c++11 -Wall -Wextra

SRCDIR = ../../../src

INCLUDE = -I$(SRCDIR) -I$(FDIR)/include

LIBRARIES = $(FDIR)/lib/libframework.a \
		    $(PDIR)/lib/libproc1.a \
		    $(PDIR)/lib/libproc2.a

CXX_SOURCES = snn-climbrate.cpp

all: snn-climbrate

# Use the Webots C API in your C++ controller program
USE_C_API = true

VERBOSE = 1

edit:
	vim snn-climbrate.cpp

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
