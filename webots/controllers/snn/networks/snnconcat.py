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


def renumber_snn(snn, offset=0):

    node_map = { i: j for  j, i in
            enumerate(sorted([int(node['id']) for node in snn['Nodes']]))
            }

    nodes_sorted = sorted(snn['Nodes'], key = lambda node: node['id'])

    nodes_renamed = [
            {'id':(node_map[node['id']] + offset), 'values':node['values'] }
            for node in nodes_sorted
            ]

    edges_renamed = [
            { 
                'from':(node_map[edge['from']] + offset), 
                'to':(node_map[edge['to']] + offset), 
                'values':edge['values'] 
                }
            for edge in snn['Edges'] ]

    return nodes_renamed, edges_renamed
 

def main():

    fmtr = argparse.ArgumentDefaultsHelpFormatter

    parser = argparse.ArgumentParser(formatter_class=fmtr)

    parser.add_argument('input_file1')

    parser.add_argument('input_file2')

    parser.add_argument('-o', '--output_file')

    args = parser.parse_args()

    snn1 = load_json(args.input_file1)

    snn1_nodes, snn1_edges = renumber_snn(snn1)

    snn1 = load_json(args.input_file2)

    snn1_max_node = [node['id'] for node in snn1_nodes][-1]

    snn2 = load_json(args.input_file2)

    new_snn2_input = snn1_max_node + 1

    snn2_nodes, snn2_edges = renumber_snn(snn2, new_snn2_input)

    snn1_outputs = snn1['Outputs']

    print(snn1_outputs)
    print(new_snn2_input)
    exit(0)

    snn_out = { 
               'Edges': snn1_edges + snn2_edges + [],
               'Nodes': snn1_nodes + snn2_nodes,
               'Associated_Data': snn1['Associated_Data'],
               'Inputs': snn1['Inputs'],
               'Network_Values': snn1['Network_Values'],
               'Outputs': snn2['Outputs'],
               'Properties': snn1['Properties']
              }

    json_out = json.dumps(snn_out, sort_keys=True, indent=4)

    if args.output_file is not None:

        with open(args.output_file, 'w') as outfile:

            outfile.write(json_out)

    else:

        print(json_out)

main()

