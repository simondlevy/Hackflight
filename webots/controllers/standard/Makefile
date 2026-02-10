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

# Command-line variable; e.g. make run world=twoexit
world = simple

WORLDS_DIR = ../../worlds

# https://stackoverflow.com/a/49524393
.EXPORT_ALL_VARIABLES:
WEBOTS_WORLD = $(world)
WEBOTS_PATH = $(shell pwd)

FILES_TO_REMOVE += *.csv __pycache__/

run:
	webots $(WORLDS_DIR)/$(world).wbt &
	/usr/local/webots/webots-controller --stdout-redirect \
		./standard.py $(world)

edit:
	vim standard.py 

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

