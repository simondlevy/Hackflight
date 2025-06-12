#include "framework_utils.hpp"

int main() 
{
    static const char * NETWORK_FILENAME =
        "/home/levys/Desktop/diffnet/difference_risp_plank.txt";

    static const double MAX = 1000;

    Processor * proc = nullptr;

    Network * net = FrameworkUtils::load(NETWORK_FILENAME, &proc);

    if (!net) {
        exit(1);
    }

    for (int j=-10; j<=+10; ++j) {

        for (int k=-10; k<+10; ++k) {

            const double inp1 = (double)j / 10;
            const double inp2 = (double)k / 10;

            const double spike_time_1 =
                FrameworkUtils::get_spike_time(inp1, MAX);

            const double spike_time_2 =
                FrameworkUtils::get_spike_time(inp2, MAX);

            vector <Spike> spikes_array = {};

            FrameworkUtils::apply_spike(net, proc, 0, spike_time_1,
                    spikes_array);
            FrameworkUtils::apply_spike(net, proc, 1, spike_time_2,
                    spikes_array);
            FrameworkUtils::apply_spike(net, proc, 2, 0,
                    spikes_array);

            const auto sim_time = 3 * MAX + 2;

            proc->run(sim_time);

            spikes_array.clear();

            const auto out = proc->output_vectors()[0][0];

            const auto time = out == MAX + 1 ? -2 : out;

            const auto diff = (time-(MAX))*4/(2*MAX)-2;

            printf("%+3.3f - %+3.3f = %+3.3f (%+3.3f)\n",
                    inp1, inp2, diff, inp1-inp2);
        }
    }
}
