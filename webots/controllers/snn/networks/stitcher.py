#!/usr/bin/python3

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


    snn_out = snn1.copy() # XXX

    json_out = json.dumps(snn_out)

    if args.output_file is not None:

        with open(args.output_file, 'w') as outfile:

            outfile.write(json_out)

    else:

        print(json_out)

main()

