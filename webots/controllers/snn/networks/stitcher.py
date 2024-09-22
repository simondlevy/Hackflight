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

    for  j, i in enumerate(sorted([int(node['id']) for node in snn1['Nodes']])):
        print(i,j)

    # print(json.dumps(snn1))

main()

