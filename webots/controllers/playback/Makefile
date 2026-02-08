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
#   make run world=simple logfile=../controller/log.csv
#
world = twoexit
logfile = ../../../simtest/log.csv

WORLDS_DIR = ../../worlds

run:
	sed 's/physics.*//' $(WORLDS_DIR)/$(world).wbt > $(WORLDS_DIR)/$(world)_playback.wbt
	webots $(WORLDS_DIR)/$(world)_playback.wbt &
	/usr/local/webots/webots-controller --stdout-redirect \
		./playback.py $(logfile)

edit:
	vim pyplayback.py

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

