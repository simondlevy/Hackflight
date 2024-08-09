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

NETWORK = $(FDIR)/networks/polebalance_risp-1.txt


cplusplus:
	webots worlds/standard.wbt &
	/usr/local/webots/webots-controller --stdout-redirect controllers/cplusplus/cplusplus

snn:
	webots worlds/standard.wbt &
	/usr/local/webots/webots-controller controllers/snn/snn

sock:

	$(FDIR)/cpp-apps/bin/polebalance_risp -a test -n $(NETWORK) \
		--all_counts_stream '{"source":"serve","port":8100}' \
		--extra_app_params '{"output":{"source":"serve","port":8888}}' &

	cd $(FDIR)/viz; love . -i '{"source":"request","port":8100,"host":"localhost"}'  -n $(NETWORK) \
		--use_name_neuron '{"2":"xd","3":"xd","4":"theta_l","5":"theta_l","6":"thetad_l","7":"thetad_l","0":"x","1":"x","9":"left","8":"no action","10":"right"}' \
		--show_spike_count --remove_unnecessary_neuron --set_num_screen_shot 1000 > /dev/null &

	cd $(FDIR)/cpp-apps/applications/polebalance/newviz; love . -v 8888 -s 1000; 

haskell:
	webots worlds/standard.wbt &
	/usr/local/webots/webots-controller --stdout-redirect controllers/haskell/haskell

