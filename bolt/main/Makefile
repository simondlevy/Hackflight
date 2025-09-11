SKETCH = main

EXTRA_SRC = vapplication.c *.cpp *.h

SRC = ../../src

include ../makefile.include

standard_config:
	cp $(SRC)/control/standard.hpp $(SRC)__control__.hpp

$(ROOT)/src/msp/__messages__.h:
	cd $(ROOT)/msppg; make install; cd $(ROOT)/teensy


