#
# Example Makefile for MSPPG Java output
#
# Copyright (C) Simon D. Levy 2015
#
# This code is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
#
# This code is distributed in the hope that it will be useful,     
# but WITHOUT ANY WARRANTY without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License 
#  along with this code.  If not, see <http:#www.gnu.org/licenses/>.

# Change this to match your desired install directory
INSTALLDIR = /home/levys/DolphinQuad/gcs/android/DolphinGCS/app/src/main/java/edu/wlu/cs/

ALL = example.class msppg.jar

all: $(ALL)

test: example.class
	java example
  
example.class: example.java
	javac example.java

jar: msppg.jar

msppg.jar:
	cd edu/wlu/cs/msppg; javac *.java
	jar cvf msppg.jar edu/wlu/cs/msppg/*.class
