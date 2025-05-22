#!/usr/bin/python3

'''
Spike-raster plotter

Copyright (C) 2025 Simon D. Levy

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, in version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
'''

from sys import stdout
import socket
from time import sleep
import threading
import json
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from btsupport import connect_to_server

SPIKE_PORT = 3

def threadfun(client, fig, spiketrains, n_neurons):

    while True:

        # Quit on plot close
        if not plt.fignum_exists(fig.number):
            break

        msg = client.recv(n_neurons)

        counts = [count for count in msg]

        # Close plot on server quit
        if len(counts) == 0:
            plt.close(fig)
            break

        for s in spiketrains:
            s['count'] = counts[s['index']]

        sleep(.001)  # yield


def animfun(frame, spiketrains, ticks, showcounts):

    for spiketrain in spiketrains:

        count = spiketrain['count']

        # Add count as a legend if indicated
        if showcounts:
            spiketrain['ax'].legend(['%d' % count], handlelength=0,
                                    loc='lower left',
                                    bbox_to_anchor=(0.01, 0.005))

        if count > 0:

            period = int(np.round(100 / count))

            lines = spiketrain['lines']

            # Add a new spike periodically
            if ticks[0] % period == 0:
                lines.append(spiketrain['ax'].plot((100, 100), (0, 1), 'k'))

            # Prune spikes as the move left outside the window
            if len(lines) > 0 and lines[0][0].get_xdata()[0] < 0:
                lines.pop(0)

            # Shift spikes to left
            for line in lines:
                ln = line[0]
                xdata = ln.get_xdata()
                ln.set_xdata(xdata - 1)

    ticks[0] += 1

    sleep(0.01)


def make_axis(ax, neuron_ids, index, is_last):

    ax.set_ylim((0, 1.1))
    ax.set_xlim((0, 100))
    ax.set_ylabel(neuron_ids[index])
    ax.set_xticks([])
    ax.set_yticks([])

    if is_last:
        ax.set_xlabel('1 sec')


def load_neuron_aliases(filename):

    # Load network from JSON file
    network = json.loads(open(filename).read())

    # Get neuron aliases by sorting node ids from network JSON
    return sorted([int(node['id']) for node in network['Nodes']])


def main():

    # Parse command-line arguments
    parser = argparse.ArgumentParser(
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('-f', '--filename',
                       help='Name of JSON-formatted network file to load')
    group.add_argument('-n', '--neuron-count', type=int,
                       help='Number of neurons')
    parser.add_argument('-i', '--ids', help='neuron ids', default='all')
    parser.add_argument('-v', '--video', help='video file to save',
                        default=None)
    parser.add_argument('-d', '--display-counts', help='display counts',
                        action='store_true')
    args = parser.parse_args()

    neuron_aliases = (load_neuron_aliases(args.filename)
                      if args.filename is not None
                      else list(range(args.neuron_count)))

    # Get desired neuron IDs to plot from command line
    neuron_ids = (neuron_aliases
                  if args.ids == 'all'
                  else list(map(int, args.ids.strip().split(','))))

    # Bozo filter
    for neuron_id in neuron_ids:
        if neuron_id not in neuron_aliases:
            print('Neuron %d not in network; quitting' % neuron_id)
            exit(1)

    # Create figure and axes in which to plot spike trains
    fig, axes = plt.subplots(len(neuron_ids))

    # Multiple neurons
    if isinstance(axes, np.ndarray):

        for k, ax in enumerate(axes):

            make_axis(ax, neuron_ids, k, k == len(axes)-1)

        # Make list of spike-train info
        spiketrains = [{'ax': ax, 'lines': [], 'count': 0,
                        'index': neuron_aliases.index(nid)}
                       for ax, nid in zip(axes, neuron_ids)]

    # Just one neuron
    else:

        make_axis(axes, neuron_ids, 0, True)

        spiketrains = [{'ax': axes, 'lines': [], 'count': 0,
                        'index': neuron_aliases.index(neuron_ids[0])}]

    # Create timestep count, to be shared between threads
    ticks = [0]

    # Connect to server
    client = connect_to_server(RPI_LOGGING_PORT)

    # Start the client thread
    thread = threading.Thread(
            target=threadfun,
            args=(client, fig, spiketrains, len(neuron_aliases)))
    thread.start()

    # Star the animation thread
    ani = animation.FuncAnimation(
            fig=fig,
            func=animfun,
            fargs=(spiketrains, ticks, args.display_counts),
            cache_frame_data=False,
            interval=1)

    plt.show()

    # Save video file if indicated
    if args.video is not None:
        print(('Saving animation to ' + args.video), end=' ... ')
        stdout.flush()
        ani.save(args.video, writer=animation.FFMpegWriter(fps=30))
        print()


main()
