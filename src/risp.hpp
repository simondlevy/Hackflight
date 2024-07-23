/* Network constants */

#include <stdint.h>
#include <stdbool.h>

#define NUM_NEURONS 512
#define NUM_SYNAPSES 512
#define NUM_OUTGOING 10
#define NUM_CHARGE_CHANGES 50
#define NUM_TRACKED_TIMESTEPS 9

class RispNetwork {

    public:

        /* Add a neuron with the given properties to the network */
        void add_neuron(int8_t threshold, uint8_t leak) {

            /* Do not add more neurons than main neurons array can hold */
            if(neuron_count >= NUM_NEURONS) return;

            /* Assign neuron properties to new neuron */

            neurons[neuron_count].num_outgoing = 0;
            neurons[neuron_count].charge = 0;
            neurons[neuron_count].threshold = threshold;
            neurons[neuron_count].leak = leak;
            neurons[neuron_count].fired = 0;
            neurons[neuron_count].check = 0;

            neuron_count++;
        }

        /* Add a synapse with the given properties to the network */
        void add_synapse(uint16_t from, uint16_t to, uint8_t delay, int8_t weight) {

            /* Ensure neuron indices are not out of range */
            if(from >= neuron_count || to >= neuron_count) return;

            /* Do not add more synapses than main synapses array can hold */
            if(synapse_count >= NUM_SYNAPSES) return;

            /* Assign synapse properties to new synapse */

            synapses[synapse_count].from = from;
            synapses[synapse_count].to = to;
            synapses[synapse_count].weight = weight;
            synapses[synapse_count].delay = delay;

            /* Add given synapse ind to given neuron's array of outgoing synapses */
            set_is_outgoing(from, synapse_count);

            synapse_count++;
        }

        /* Set whether or not the network has leak */
        void set_no_leak(uint8_t has_no_leak) {
            no_leak = has_no_leak & 1;
        }

        /* Set whether or not the network allows negative charge values */
        void set_non_negative_charge(uint8_t has_non_negative_charge) {
            non_negative_charge = has_non_negative_charge & 1;
        }

        /* Set whether or not neurons should fire like RAVENS (where neurons fire the timestep after charge exceeds threshold) */
        void set_fire_like_ravens(uint8_t should_fire_like_ravens) {
            fire_like_ravens = should_fire_like_ravens & 1;
        }

        /* Queue spike as input to a network */
        void apply_spike(uint16_t neuron_ind) {

            /* Ensure neuron index is not out of range */
            if(neuron_ind >= neuron_count) return;

            /* Schedule charge change at current timestep for neuron with given index */
            if(event_count[cur_charge_changes_ind] < NUM_CHARGE_CHANGES) {
                charge_changes[cur_charge_changes_ind][event_count[cur_charge_changes_ind]].neuron_ind = neuron_ind;
                charge_changes[cur_charge_changes_ind][event_count[cur_charge_changes_ind]].charge_change = 1.0;
                event_count[cur_charge_changes_ind]++;
            }
        }

        /* Run the network for one timestep with queued input(s) */
        void run() {

            uint16_t neuron_ind;

            /* Reset all neurons' fired flag  */
            for(neuron_ind = 0; neuron_ind < neuron_count; neuron_ind++) {
                neurons[neuron_ind].fired = 0;
            }

            process_events();

            /* Prepare for next timestep by "shifting" charge changes array back by one timestep */
            event_count[(cur_charge_changes_ind + NUM_TRACKED_TIMESTEPS) % NUM_TRACKED_TIMESTEPS] = 0;

            if(cur_charge_changes_ind < NUM_TRACKED_TIMESTEPS-1) {
                cur_charge_changes_ind += 1;
            } else {
                cur_charge_changes_ind = 0;
            }
        }

        /* See if neuron fired in the most recent timestep */
        uint8_t fired(uint16_t neuron_ind) {

            /* Ensure neuron index is not out of range */
            if(neuron_ind >= neuron_count) return 0;

            /* Return 1 if neuron fired in most recent timestep, 0 otherwise */
            return neurons[neuron_ind].fired;
        }
        typedef struct {
            uint16_t from;  /* Index of from neuron */
            uint16_t to;    /* Index of to neuron */
            uint8_t delay;  /* Synapse delay value */
            int8_t weight;  /* Synapse weight value */
        } Synapse;

        /* Charge change event struct (essentially just a pair) */
        typedef struct {
            uint16_t neuron_ind;    /* Index of neuron to change the charge for */
            int8_t charge_change;   /* Value to change charge by */
        } Charge_Change_Event;


    private:

        /* Neuron struct */
        typedef struct {
            /* Synapses */

            uint16_t outgoing[NUM_OUTGOING];    /* Indices of outgoing synapses */
            uint8_t num_outgoing;               /* Number of outgoing synapses for this neuron */

            /* Neuron properties and associated state */

            int8_t charge;     /* Charge value */
            int8_t threshold;   /* Threshold value */
            uint8_t leak;       /* Leak value (1 for full leak and 0 for no leak) */
            uint8_t fired;      /* Did neuron fire in most recent timestep */
            uint8_t check;      /* Does neuron need to be checked for a fire still */
        } Neuron;


        Neuron neurons[NUM_NEURONS];     /* Array of all neurons */
        Synapse synapses[NUM_SYNAPSES];  /* Array of all synapses */

        uint16_t neuron_count = 0;   /* Number of neurons in network */
        uint16_t synapse_count = 0;  /* Number of synapses in network */

        uint16_t to_fire[NUM_NEURONS];   /* Neuron indices for neurons that need to be fired at the beginning of the upcoming timestep */
        uint16_t to_fire_count = 0;      /* Number of neurons that need to be fired at the beginning of the upcoming timestep */

        Charge_Change_Event charge_changes[NUM_TRACKED_TIMESTEPS][NUM_CHARGE_CHANGES];   /* Charge changes keyed on timestep and charge change event index */
        uint8_t event_count[NUM_TRACKED_TIMESTEPS] = {0, 0, 0, 0, 0, 0, 0, 0, 0};        /* Number of charge change events for each upcoming timestep */
        uint8_t cur_charge_changes_ind = 0;                                              /* Index of charge changes array that represents which array of charge change events corresponds to the upcoming timestep */

        uint8_t no_leak;             /* 1 means whole network has no leak */
        uint8_t non_negative_charge; /* 1 means negative charge is not allowed throughout network */
        uint8_t fire_like_ravens;    /* 1 means neuron fires occur the timestep after neuron charge exceeds its threshold */


        /* Private Network procedures */

        /* Add given synapse ind to given neuron's array of outgoing synapses */
        void set_is_outgoing(uint16_t neuron_ind, uint16_t synapse_ind) {

            /* Abort if the given neuron already has the max number of outgoing synapses */
            if(neurons[neuron_ind].num_outgoing >= NUM_OUTGOING) {
                return;
            }

            neurons[neuron_ind].outgoing[neurons[neuron_ind].num_outgoing] = synapse_ind;
            neurons[neuron_ind].num_outgoing++;
        }

        /* Make the given neuron fire */
        void perform_fire(uint16_t neuron_ind) {

            /* Set flag indicating neuron fired this timestep */
            neurons[neuron_ind].fired = 1;

            /* Reset neuron charge */
            neurons[neuron_ind].charge = 0;
        }

        /* Process events at the current timestep */
        void process_events() {

            uint16_t to_fire_ind;
            uint8_t charge_change_ind;
            uint16_t neuron_ind;
            uint16_t synapse_ind;
            uint8_t out_synapse_ind;
            uint8_t to_time;

            /* If RISP is configured to fire like RAVENS, then fire neurons with threshold exceeding charge at beginning of timestep */
            for(to_fire_ind = 0; to_fire_ind < to_fire_count; to_fire_ind++) {
                perform_fire(to_fire[to_fire_ind]);
            }
            to_fire_count = 0;

            /* Apply neuron leak when leak is enabled (one of neurons has non-zero leak) */
            if(no_leak == 0) {
                for(charge_change_ind = 0; charge_change_ind < event_count[cur_charge_changes_ind]; charge_change_ind++) {
                    neuron_ind = charge_changes[cur_charge_changes_ind][charge_change_ind].neuron_ind;
                    if(neurons[neuron_ind].leak || (non_negative_charge && neurons[neuron_ind].charge < 0)) {
                        neurons[neuron_ind].charge = 0;
                    }
                }
            }

            /* Collect charges */
            for(charge_change_ind = 0; charge_change_ind < event_count[cur_charge_changes_ind]; charge_change_ind++) {
                neuron_ind = charge_changes[cur_charge_changes_ind][charge_change_ind].neuron_ind;
                neurons[neuron_ind].check = 1;
                neurons[neuron_ind].charge += charge_changes[cur_charge_changes_ind][charge_change_ind].charge_change;
            }

            /* Determine which neurons to fire */
            for(charge_change_ind = 0; charge_change_ind < event_count[cur_charge_changes_ind]; charge_change_ind++) {

                neuron_ind = charge_changes[cur_charge_changes_ind][charge_change_ind].neuron_ind;
                if(neurons[neuron_ind].check) {

                    /* Fire the neuron if its charge >= its threshold */
                    if(neurons[neuron_ind].charge >= neurons[neuron_ind].threshold) {

                        /* Schedule charge changes due to spiking */
                        for(out_synapse_ind = 0; out_synapse_ind < neurons[neuron_ind].num_outgoing; out_synapse_ind++) {
                            synapse_ind = neurons[neuron_ind].outgoing[out_synapse_ind];
                            to_time = (cur_charge_changes_ind + synapses[synapse_ind].delay) % NUM_TRACKED_TIMESTEPS;

                            /* Schedule the charge change if there is space for another event in that timestep */
                            if(event_count[to_time] < NUM_CHARGE_CHANGES) {
                                charge_changes[to_time][event_count[to_time]].neuron_ind = synapses[synapse_ind].to;
                                charge_changes[to_time][event_count[to_time]].charge_change = synapses[synapse_ind].weight;
                                event_count[to_time]++;
                            }
                        }

                        if(fire_like_ravens) {
                            to_fire[to_fire_count] = neuron_ind;
                            to_fire_count += 1;
                        } else {
                            perform_fire(neuron_ind);
                        }
                    }
                    neurons[neuron_ind].check = 0;
                }
            }

        }
};
