#!/usr/bin/python3

import json
import argparse


def main():

    fmtr = argparse.ArgumentDefaultsHelpFormatter

    parser = argparse.ArgumentParser(formatter_class=fmtr)

    parser.add_argument('file1')

    parser.add_argument('file2')

    args = parser.parse_args()

main()

