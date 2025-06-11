FRO = $(HOME)/Desktop/framework-open

CXX ?= g++

FR_INC = $(FRO)/include/framework.hpp
FR_CFLAGS = -std=c++11 -Wall -Wextra -I$(FRO)/include -I$(FRO)/include/utils $(CFLAGS)
FR_OBJ = obj/framework.o obj/processor_help.o obj/properties.o

RISP_INC = $(FRO)/include/risp.hpp
RISP_OBJ = obj/risp.o obj/risp_static.o

all: processor_tool_risp

processor_tool_risp: processor_tool.cpp $(FR_INC) $(RISP_INC) $(RISP_OBJ) $(FR_OBJ)
	$(CXX) $(FR_CFLAGS) -o processor_tool_risp $(FRO)/src/processor_tool.cpp $(RISP_OBJ) $(FR_OBJ)

obj/risp.o: $(FRO)/src/risp.cpp $(FR_INC) $(RISP_INC)
	$(CXX) -c $(FR_CFLAGS) -o obj/risp.o $(FRO)/src/risp.cpp

obj/risp_static.o: $(FRO)/src/risp_static.cpp $(FR_INC) $(RISP_INC)
	$(CXX) -c $(FR_CFLAGS) -o obj/risp_static.o $(FRO)/src/risp_static.cpp

obj/framework.o: $(FRO)/src/framework.cpp $(FR_INC)
	$(CXX) -c $(FR_CFLAGS) -o obj/framework.o $(FRO)/src/framework.cpp

obj/processor_help.o: $(FRO)/src/processor_help.cpp $(FR_INC)
	$(CXX) -c $(FR_CFLAGS) -o obj/processor_help.o $(FRO)/src/processor_help.cpp

obj/properties.o: $(FRO)/src/properties.cpp $(FR_INC)
	$(CXX) -c $(FR_CFLAGS) -o obj/properties.o $(FRO)/src/properties.cpp

clean:
	rm -f obj/*


