'''
Generic dialog code for Hackflight GCS

Copyright (C) Simon D. Levy 2021

MIT License
'''


class Dialog(object):

    def __init__(self, gcs):

        self.running = False

        self.gcs = gcs

        self.width = int(self.gcs.canvas['width'])
        self.height = int(self.gcs.canvas['height'])

        self.gcs.root.bind("<Key>", self._check_quit)

    def hide(self, widget):

        self.gcs.hide(widget)

    def schedule_display_task(self, delay_msec):

        self.gcs.scheduleTask(delay_msec, self._task)

    def delete(self, widget):

        self.gcs.canvas.delete(widget)

    def place(self, widget, x, y):

        widget.place(x=x, y=y, width=self.width, height=self.height)

    def start(self):

        self.running = True

    def stop(self):

        self.running = False

    def _check_quit(self, event):

        if ord(event.char) == 27:  # ESC
            exit(0)
