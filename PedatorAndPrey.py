# -*- coding: utf-8 -*-

from GraphicsLib import *
import numpy as np


class Predator(Particle):
    '''
    A repeller is a particle which is attracted to attractor particles only
    and repels itself from other Predators when in close range. 
    '''
    def __init__(self, scene, position, velocity=np.zeros(2), **kwargs):
        super().__init__(scene, position, velocity=velocity, **kwargs)
        self.maxspeed = 5.
        self.maximumforce = .2
        self.killradius = 10
        self.info = True
        self.textid = None
        
    def update(self):
        desired = np.array([0.,0.])
        kill_list = []
        for p in self.scene.particles:
            if type(p) is Prey:
                r = (p.position - self.position)
                mag = np.sqrt(np.sum(r**2))
                if mag != 0:  
                    desired += (1. / mag**2) * (r / mag)
                if mag < self.killradius:
                    kill = self.scene.particles.index(p)
                    kill_list.append(kill)
            elif type(p) is Predator and not p == self:
                r = (p.position - self.position)
                mag = np.sqrt(np.sum(r**2))
                if mag != 0:  
                    desired -= (1. / (mag**3)) * (r / mag)
        for kill in kill_list:
            part = self.scene.particles.pop(kill)
            self.scene.delete(part.id)
        m = np.sqrt(np.sum(desired**2))
        if m != 0:
            desired = self.maxspeed * desired / m 
        force = desired - self.velocity
        fm = np.sqrt(np.sum(force**2))
        if fm > self.maximumforce:
            force = self.maximumforce * force / fm
        self.addforce(force)
        self.update_motion()
        #wrap around borders
        #self.position = self.position % np.array([self.scene.winfo_width(), self.scene.winfo_height()])
        
    def render(self, length, **kwargs):
        mag = np.sqrt(np.sum(self.velocity**2))
        if mag == 0:
            norm = np.array([1.,0.])
        else:
            norm = self.velocity / mag
        start = self.position - length / 2 * norm
        end = self.position + length / 2 * norm
        id = self.scene.segment(start, end, arrow="last", **kwargs)
        return id
    
    def displayinfo(self, info):
        self.scene.delete(self.textid)
        self.textid = self.scene.create_text(self.position[0], self.position[1] - 10, text=str(info))
    
class Prey(Particle):
    '''
    A Prey repels itself from all other particles.
    '''
    def __init__(self, scene, position, velocity=np.zeros(2), **kwargs):
        super().__init__(scene, position, velocity=velocity, **kwargs)
        self.maxspeed = 4.5
        self.textid = None
        self.vectorid = None
        self.arrows = False
        self.info = False
    
    def update(self):
        desired = np.array([0.,0.])
        for p in self.scene.particles:
            if not p == self:
                r = (p.position - self.position)
                mag = np.sqrt(np.sum(r**2))
                if mag != 0:  
                    desired -= 5. / mag * (r / mag)
        self.addforce(desired)
        # borders
        dx1 = (self.scene.winfo_width() - self.position[0]) / self.scene.winfo_width()
        dx2 = (0. - self.position[0]) / self.scene.winfo_width()
        dy1 = (self.scene.winfo_height() - self.position[1]) / self.scene.winfo_height()
        dy2 = (0. - self.position[1]) / self.scene.winfo_height()
        boundary1 = 1. / dx1 + 1. / dx2
        boundary2 = 1. / dy1 + 1. / dy2
        self.addforce(.01 * np.array([-boundary1, -boundary2]))
        # text info
        if self.info:
            self.displayinfo()
        if self.arrows:
            self.displayarrows()
        self.update_motion()
        # wrap around borders
        #self.position = self.position % np.array([self.scene.winfo_width(), self.scene.winfo_height()])
        v = np.sqrt(np.sum(self.velocity**2))
        if v > self.maxspeed:
            self.velocity = self.maxspeed * self.velocity / v 
        
    def displayinfo(self):
        self.scene.delete(self.textid)
        self.textid = self.scene.create_text(self.position[0], self.position[1] - 10, text=str(self.position))
    
    def displayarrows(self):
        self.scene.delete(self.vectorid)
        self.vectorid = self.scene.segment(self.position , self.position + (500 * self.acceleration), arrow="last")
        

def main():
   app = App()
   scene = Scene(app.mainframe, width=900, height=600)
   scene.deltatime = 20
   
   for i in range(25):
      prey = Prey(scene, 600 * np.random.rand(2) + 10, radius=5)
      scene.addparticle(prey)
   for i in range(7):
       predator = Predator(scene, 600 * np.random.rand(2) + 10, length=20)
       scene.addparticle(predator)
   app.mainloop()

if __name__ == '__main__':
    main()
    