#  Copyright (C) 2026 Simon D. Levy
 
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, in version 3.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program. If not, see <http:--www.gnu.org/licenses/>.

# Command-line variables; e.g.:
#   make run world=simple log=../controller/log.csv
#
world = simple
log = ../standard/log.csv

WORLDS_DIR = ../../worlds
WORLD_FILE = $(WORLDS_DIR)/$(world).wbt
PLAYBACK_FILE = $(WORLDS_DIR)/playback_$(world).wbt

run:
	@if [ ! -f $(WORLD_FILE) ]; then\
        echo "World file" $(WORLD_FILE) "not found";\
	elif [ ! -f $(log) ]; then\
        echo "Log file" $(log) "not found";\
	else \
		echo "Okay";\
		sed 's/physics.*//' $(WORLD_FILE) > $(PLAYBACK_FILE);\
	    webots $(PLAYBACK_FILE) & /usr/local/webots/webots-controller --stdout-redirect ./playback.py $(log);\
	fi

edit:
	vim playback.py

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

