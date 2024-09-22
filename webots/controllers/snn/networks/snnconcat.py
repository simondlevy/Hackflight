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

    snn2 = load_json(args.input_file2)

    snn1map = { i: j for  j, i in
            enumerate(sorted([int(node['id']) for node in snn1['Nodes']]))
            }

    nodes1_sorted = sorted(snn1['Nodes'], key = lambda node: node['id'])

    nodes1_renamed = [
            {'id':snn1map[node['id']], 'values':node['values'] }
            for node in nodes1_sorted
            ]

    edges1_renamed = [
            { 
                'from':snn1map[edge['from']], 
                'to':snn1map[edge['to']], 
                'values':edge['values'] 
                }
            for edge in snn1['Edges'] ]

    snn_out = { 
               'Edges': edges1_renamed,
               'Nodes': nodes1_renamed,
               'Associated_Data': snn1['Associated_Data'],
               'Inputs': snn1['Inputs'],
               'Network_Values': snn1['Network_Values'],
               'Outputs': snn1['Outputs'],
               'Properties': snn1['Properties']
              }

    json_out = json.dumps(snn_out, sort_keys=True, indent=4)

    if args.output_file is not None:

        with open(args.output_file, 'w') as outfile:

            outfile.write(json_out)

    else:

        print(json_out)

main()

