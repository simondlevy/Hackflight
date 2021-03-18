'''
Resource access to support for GCS

Copyright (C) Simon D. Levy 2021

MIT License
'''

import os


def resource_path(filename):

    return os.path.join('media', filename)
