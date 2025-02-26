NETWORK = difference_risp_train.txt

IPADDR = 137.113.118.30
# IPADDR = 10.42.0.1

PDIR =  $(FDIR)/processors/risp

spikeplot:
	$(HOME)/Desktop/spikeplotter/spikeplot.py -f $(HOME)/Desktop/framework/networks/$(NETWORK) \
		--server $(IPADDR) --ids 0,1,2 --display-counts
