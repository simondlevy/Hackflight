#!/usr/bin/python3

'''
Copyright (C) 2024 Simon D. Levy

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, in version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http:--www.gnu.org/licenses/>.
'''

import json
import argparse


def load_json(filename):

    return json.loads(open(filename).read())


def main():

    fmtr = argparse.ArgumentDefaultsHelpFormatter

    parser = argparse.ArgumentParser(formatter_class=fmtr)

    parser.add_argument('input_file1')

    parser.add_argument('input_file2')

    parser.add_argument('-o', '--output_file')

    args = parser.parse_args()

    snn1 = load_json(args.input_file1)
    snn1_nodes = snn1['Nodes']
    snn1_edges = snn1['Edges']

    snn2 = load_json(args.input_file2)
    snn2_nodes = snn2['Nodes']
    snn2_edges = snn2['Edges']

    offset = max([node['id'] for node in snn1_nodes]) + 1

    snn2 = load_json(args.input_file2)

    snn2_nodes_renumbered = [
            {'id': node['id'] + offset, 'values': node['values']}
            for node in snn2_nodes
            ]

    snn2_edges_renumbered = [
            {
                'from': edge['from'] + offset,
                'to': edge['to'] + offset,
                'values': edge['values']
                }
            for edge in snn2['Edges']
            ]

    snn2_inputs_renumbered = list(map(lambda n : n + offset, snn2['Inputs']))

    snn2_outputs_renumbered = list(map(lambda n : n + offset, snn2['Outputs']))

    snn1_outputs = snn1['Outputs']

    new_edges = [
            {'from': node,
             'to': snn2_inputs_renumbered[0],
             'values': [1, 0]}
            for node in snn1_outputs]

    snn_out = {
               'Edges': snn1_edges + snn2_edges_renumbered + new_edges,
               'Nodes': snn1_nodes + snn2_nodes_renumbered,
               'Inputs': snn1['Inputs'] + snn2_inputs_renumbered[1:],
               'Outputs': snn2_outputs_renumbered,
               'Network_Values': snn1['Network_Values'],
               'Associated_Data': snn1['Associated_Data'],
               'Properties': snn1['Properties']
              }

    json_out = json.dumps(snn_out, sort_keys=True, indent=4)

    if args.output_file is not None:

        with open(args.output_file, 'w') as outfile:

            outfile.write(json_out)

    else:

        print(json_out)


main()
