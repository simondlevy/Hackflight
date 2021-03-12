#
# Example Makefile for MSPPG Java output
#
# Copyright (C) Simon D. Levy 2021
#
# MIT License

ALL = msppg.jar

all: $(ALL)

test: example.class
	java example
  
example.class: example.java
	javac example.java

jar: msppg.jar

msppg.jar:
	cd edu/wlu/cs/msppg; javac *.java
	jar cvf msppg.jar edu/wlu/cs/msppg/*.class

clean:
	rm -f msppg.jar

copy: msppg.jar
	cp msppg.jar ../../../debug/java/
