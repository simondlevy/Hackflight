'''
Tkinter/tkinter compatability to support both Python2 and Python3

Copyright (C) Simon D. Levy 2021

MIT License
'''


try:
    from Tkinter import *
except:
    from tkinter import *
