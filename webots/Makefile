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

FDIR = $(HOME)/Desktop/framework

# DIFFERENCE_NETWORK = $(FDIR)/networks/difference_risp.txt
DIFFERENCE_NETWORK = $(HOME)/Desktop/hackflight/webots/controllers/snn-climbrate/climbrate_risp.txt

VIZ_HOST = "localhost"
VIZ_PORT = 8100

cplusplus:
	webots worlds/simple.wbt &
	/usr/local/webots/webots-controller --stdout-redirect controllers/cplusplus/cplusplus

haskell:
	webots worlds/simple.wbt &
	/usr/local/webots/webots-controller --stdout-redirect controllers/haskell/haskell

snn-climbrate:
	@webots worlds/simple.wbt &
	@/usr/local/webots/webots-controller controllers/snn-climbrate/snn-climbrate $(DIFFERENCE_NETWORK) &
	@sleep 1
	@cd $(FDIR)/viz; love . -i '{"source":"request","port":$(VIZ_PORT),"host":$(VIZ_HOST)}'  -n $(DIFFERENCE_NETWORK) \

		--show_spike_count --remove_unnecessary_neuron --set_num_screen_shot 0 > /dev/null
