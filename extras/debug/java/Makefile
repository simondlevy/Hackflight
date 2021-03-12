#
# Makefile for MSPPG Java example
#
# Copyright (C) Simon D. Levy 2021
#
# MIT License

ALL = StateParser.class

all: $(ALL)

test: StateParser.class
	java -classpath msppg.jar:. StateParser
  
StateParser.class: StateParser.java
	javac -classpath msppg.jar StateParser.java

clean:
	rm -rf $(ALL)
