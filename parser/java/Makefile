#   Copyright (c) 2022 Simon D. Levy
#
#   This file is part of Hackflight.
#
#   Hackflight is free software: you can redistribute it and/or modify it under the
#   terms of the GNU General Public License as published by the Free Software
#   Foundation, either version 3 of the License, or (at your option) any later
#   version.
#
#   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
#   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
#   PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License along with
#   Hackflight. If not, see <https://www.gnu.org/licenses/>.

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
