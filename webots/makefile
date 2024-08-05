#  Copyright (C) 2024 Simon D. Levy
 
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

cplusplus:
	webots worlds/standard.wbt &
	/usr/local/webots/webots-controller --stdout-redirect controllers/cplusplus/cplusplus

snn:
	webots worlds/standard.wbt &
	/usr/local/webots/webots-controller controllers/snn/snn

haskell:
	webots worlds/standard.wbt &
	/usr/local/webots/webots-controller --stdout-redirect controllers/haskell/haskell

python:
	webots worlds/standard.wbt &
	/usr/local/webots/webots-controller --stdout-redirect controllers/python/python.py

pytakeoff:
	webots worlds/standard.wbt &
	/usr/local/webots/webots-controller controllers/pytakeoff/pytakeoff.py

playback-motors:
	webots worlds/standard.wbt &
	/usr/local/webots/webots-controller controllers/playback-motors/playback-motors.py

playback-demands:
	webots worlds/standard.wbt &
	/usr/local/webots/webots-controller controllers/playback-demands/playback-demands.py

playback-pose:
	webots worlds/supervised.wbt &
	/usr/local/webots/webots-controller controllers/playback-pose/playback-pose.py
