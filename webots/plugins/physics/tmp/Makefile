NAME = difference_risp

FRO = $(HOME)/Desktop/framework-open

CXX ?= g++ -O3 -std=c++11 -Wall -Wextra

FR_INC = -I$(FRO)/include -I$(FRO)/include/utils $(CFLAGS)
FR_OBJ = obj/framework.o obj/processor_help.o obj/properties.o

RISP_OBJ = obj/risp.o obj/risp_static.o

all: difference_risp

$(NAME): $(NAME).cpp $(RISP_OBJ) $(FR_OBJ)
	$(CXX) $(FR_INC) -o $(NAME) $(NAME).cpp $(RISP_OBJ) $(FR_OBJ)

obj/risp.o:
	$(CXX) -c $(FR_INC) -o obj/risp.o $(FRO)/src/risp.cpp

obj/risp_static.o:
	$(CXX) -c $(FR_INC) -o obj/risp_static.o $(FRO)/src/risp_static.cpp

obj/framework.o:
	$(CXX) -c $(FR_INC) -o obj/framework.o $(FRO)/src/framework.cpp

obj/processor_help.o:
	$(CXX) -c $(FR_INC) -o obj/processor_help.o $(FRO)/src/processor_help.cpp

obj/properties.o:
	$(CXX) -c $(FR_INC) -o obj/properties.o $(FRO)/src/properties.cpp

run: $(NAME)
	./$(NAME)

edit:
	vim $(NAME).cpp

clean:
	rm -f $(NAME) obj/*


