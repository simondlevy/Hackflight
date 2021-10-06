#
# Example Makefile for MSPPG Java output
#
# Copyright (C) Simon D. Levy 2021
#
# MIT License

ALL = msp.jar

all: $(ALL)

jar: msp.jar

msp.jar:
	cd edu/wlu/cs/msp; javac *.java
	jar cvf msp.jar edu/wlu/cs/msp/*.class

clean:
	rm -f msp.jar

copy: msp.jar
	cp msp.jar ../../../debug/java/
