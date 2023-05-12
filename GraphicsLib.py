# -*- coding: utf-8 -*-
"""
Created on Wed Jan 26 12:29:19 2022

@author: Mark
"""

from tkinter import *
from tkinter import ttk
import numpy as np

class App(Tk):
    def __init__(self):
        Tk.__init__(self)
        self.title("Test Application")
        self.mainframe = ttk.Frame(self, padding="3 3 12 12")
        self.mainframe.grid()
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        self.protocol("WM_DELETE_WINDOW", self.on_exit)
        
    def on_exit(self):
        self.destroy()
        id_list = self.tk.call('after', 'info')
        for id in id_list:
            self.after_cancel(id)
     

class Scene(Canvas):
    '''
    An extension of a Tkinter canvas where parent is the root or some other frame.
    Contains the functionality to add custom geometry as well as a universal
    particle system.
    '''
    def __init__(self, parent, column=0, row=0, deltatime=16, **kwargs):
        super().__init__(parent, **kwargs)
        self.grid(column=column, row=row)
        self.deltatime = deltatime
        self.particles = []
        self.after(self.deltatime, self.updateloop)
        
    def updateloop(self):
        '''
        Calls the update method of all particles in the particle list
        '''
        for p in self.particles:
            p.update()
        self.after(self.deltatime, self.updateloop )
        
    def addparticle(self, particle):
        '''
        Add a particle, which inherits from the Particle base class, to the particle list
        '''
        #p = particle(self, pos, **kwargs)
        self.particles.append(particle)
        #return p
    
    def segment(self, p1, p2, **kwargs):
        '''
        Creates a line segment between two points p1 and p2 (xcoord, ycoord)
        '''
        id = self.create_line(p1[0], p1[1], p2[0], p2[1], **kwargs)
        return id
    
    def circle(self, c, radius, **kwargs):
        '''
        Creates a circle defined by its center c (xcoord, ycoord) and a radius.
        '''
        id = self.create_oval(c[0] - radius, c[1] - radius, c[0] + radius, c[1] + radius, **kwargs)
        return id
    
 
class Particle:
    '''
    The base class of a particle. Defaults to an circle with a constant velocity.
    To create custom particle/agents inherit this class and overwrite its update
    method, which is called every frame by the specified Scene object.
    '''
    def __init__(self, scene, position, velocity=np.zeros(2), **kwargs):
        self.scene = scene
        self.position = position
        self.velocity = velocity
        self.acceleration = np.zeros(2)
        self.kwargs = kwargs
        self.id = None #self.render(**kwargs)
        self.info = True
        self.textid = None
        self.enableinfo = False
     
    def render(self, **kwargs):
        id = self.scene.circle(self.position, **kwargs)
        return id
        
    def update(self):
        self.update_motion()
        
    def update_motion(self):
        self.velocity = self.velocity + self.acceleration
        self.position = self.position + self.velocity
        self.acceleration *= 0
        if type(self.id) is list:
            for i in self.id:
                self.scene.delete(i)
        else:
            self.scene.delete(self.id)
        self.id = self.render(**self.kwargs)
        
    def addforce(self, force):
        self.acceleration += force;
        
    def displayinfo(self, info, **kwargs):
        self.scene.delete(self.textid)
        self.textid = self.scene.create_text(self.position[0], self.position[1] - 10, text=str(info), **kwargs)
    
    def normalize(self, v):
        norm = np.linalg.norm(v)
        if norm == 0: 
           return v
        return v / norm
        
        
def testing():
    
    app = App()
    scene = Scene(app.mainframe, width=600, height=400)
    particle = Particle(scene, np.array([100,100]), velocity=np.array([1,1]), radius=10)
    scene.addparticle(particle)
    app.mainloop()


if __name__ == '__main__':
    testing()