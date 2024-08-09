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

dzplot:
	webots worlds/standard.wbt &
	/usr/local/webots/webots-controller --stdout-redirect controllers/cplusplus/cplusplus | ./dzplot.py

sock:
	webots worlds/standard.wbt &
	/usr/local/webots/webots-controller controllers/sock-snn/sock-snn

snn:
	webots worlds/standard.wbt &
	/usr/local/webots/webots-controller controllers/snn/snn

takeoff:
	webots worlds/standard.wbt &
	/usr/local/webots/webots-controller controllers/takeoff/takeoff

haskell:
	webots worlds/standard.wbt &
	/usr/local/webots/webots-controller --stdout-redirect controllers/haskell/haskell

