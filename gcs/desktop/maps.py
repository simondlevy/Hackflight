#!/usr/bin/env python

URLBASE = 'https://maps.googleapis.com/maps/api/staticmap?center=%f,%f&zoom=%d&size=%dx%d&maptype=%s'


LATITUDE  =  37.7913838
LONGITUDE = -79.44398934
ZOOM      = 15
MAPTYPE   = 'roadmap'

from dialog import Dialog
from goompy import GooMPy

import Tkinter
import PIL.ImageTk

class Maps(Dialog):

    def __init__(self, driver, yoffset=0):

        Dialog.__init__(self, driver)

        self.label = Tkinter.Label(self.driver.canvas)

        self.radiogroup = Tkinter.Frame(self.driver.canvas)
        self.radiovar = Tkinter.IntVar()
        self.maptypes = ['roadmap', 'terrain', 'satellite', 'hybrid']
        self._add_radio_button('Road Map',  0)
        self._add_radio_button('Terrain',   1)
        self._add_radio_button('Satellite', 2)
        self._add_radio_button('Hybrid',    3)

        self.zoom_in_button  = self._add_zoom_button('+', +1)
        self.zoom_out_button = self._add_zoom_button('-', -1)

        self.zoomlevel = ZOOM

        maptype_index = 0
        self.radiovar.set(maptype_index)

        self.goompy = GooMPy(self.width, self.height, LATITUDE, LONGITUDE, ZOOM, MAPTYPE)

 
    def start(self):

        self.driver.root.bind('<B1-Motion>', self._drag)
        self.driver.root.bind('<Button-1>', self._click)

        self._redraw()

        self.running = True

    def stop(self):

        self.driver.root.unbind('<B1-Motion>')
        self.driver.root.unbind('<Button-1>')

        self.hide(self.radiogroup)
        self.hide(self.label)
        self.hide(self.zoom_in_button)
        self.hide(self.zoom_out_button)
        self.running = False

    def _add_zoom_button(self, text, sign):

        button = Tkinter.Button(self.driver.canvas, text=text, width=1, command=lambda:self._zoom(sign))
        return button

    def _zoom(self, sign):

        newlevel = self.zoomlevel + sign
        if newlevel > 0 and newlevel < 22:
            self.zoomlevel = newlevel
            self.goompy.useZoom(newlevel)
            self._redraw()


    def _add_radio_button(self, text, index):

        maptype = self.maptypes[index]
        Tkinter.Radiobutton(self.radiogroup, text=maptype, variable=self.radiovar, value=index, 
                command=lambda:self._usemap(maptype)).grid(row=0, column=index)
 
    def _click(self, event):

        self.coords = event.x, event.y

    def _drag(self, event):

        self.goompy.move(self.coords[0]-event.x, self.coords[1]-event.y)
        self.image = self.goompy.getImage()
        self._redraw()
        self.coords = event.x, event.y

    def _redraw(self):

        self.image = self.goompy.getImage()
        self.image_tk = PIL.ImageTk.PhotoImage(self.image)
        self.label['image'] = self.image_tk
        self.label.place(x=0, y=0, width=self.width, height=self.height) 

        self.radiogroup.place(x=0,y=0)

        x = self.width - 50
        y = self.height - 130
        self.zoom_in_button.place(x= x, y=y)
        self.zoom_out_button.place(x= x, y=y+30)

    def _usemap(self, maptype):

        self.goompy.useMaptype(maptype)
        self._redraw()

class _MapsDriver(object):

    def __init__(self):

        self.root = Tkinter.Tk()
        self.root.geometry('%dx%d' % (800,500))
        self.root.title('Maps')

        self.canvas = Tkinter.Canvas(self.root, width=800, height=500, background='black')

        self.canvas.pack()
        
        self.maps = Maps(self)

        self.maps.start()

        self.root.mainloop() 

if __name__ == '__main__':

    _MapsDriver()

    
