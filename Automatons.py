# -*- coding: utf-8 -*-
"""
Created on Tue Sep 20 13:25:07 2022

@author: Mark
"""

from GraphicsLib import *
import numpy as np


class Vehicle(Particle):
    '''
    
    '''
    def __init__(self, scene, position, velocity=np.zeros(2), **kwargs):
        super().__init__(scene, position, velocity=velocity, **kwargs)
        self.maxspeed = 4.
        self.minspeed = 3.
        self.maximumforce = .2
        self.wanderdirection = np.ones(2)
        self.vectorid = None
        
    def update_refined_motion(self, force, wrapborder=False):
        f = np.sqrt(np.sum(force**2))
        if f > self.maximumforce:
            force = self.maximumforce * self.normalize(force)
        v = np.sqrt(np.sum(self.velocity**2))
        if v > self.maxspeed:
            self.velocity = self.maxspeed * self.normalize(self.velocity)
        #if v < self.minspeed and not v == 0:
            #self.velocity = self.minspeed * self.normalize(self.velocity)
        self.addforce(force)
        self.update_motion()
        if wrapborder:
            self.position = self.position % np.array([self.scene.winfo_width(), self.scene.winfo_height()])
    
    def seek(self, target, velocity):
        desired = self.normalize(target - self.position) * velocity
        return self.normalize(desired - self.velocity)
    
    def flee(self, target, velocity):
        desired = self.normalize(self.position - target) * velocity
        return desired - self.velocity
    
    def pursue(self, target, velocity, c=.7):
        t = 0.0
        if not velocity==0:
            t = np.linalg.norm(target.position - self.position) / velocity
        future = target.position + target.velocity * t * c
        return self.seek(future, velocity)
    
    def offsetpursue(self, target, offset, velocity, c=.7):
        t = 0.0
        if not velocity==0:
            t = np.linalg.norm(target.position - self.position) / velocity
        future = target.position + target.velocity * t * c
        return None
    
    def evade(self, target, velocity, c=.7):
        t = 0.0
        if not velocity==0:
            t = np.linalg.norm(target.position - self.position) / velocity
        future = target.position + target.velocity * t * c
        return self.flee(future, velocity)
    
    def arrive(self, target, slowing_distance,  velocity):
        distance = np.linalg.norm(target - self.position)
        if distance == 0:
            return np.zeros(2) - self.velocity
        ramped_speed = velocity * (distance / slowing_distance)
        clipped_speed = min(ramped_speed, velocity)
        desired = min(clipped_speed / distance, velocity) * (target - self.position)
        return desired - self.velocity
    
    def wander(self, rate, forward, velocity):
        newdir = self.normalize(self.wanderdirection + self.normalize(np.random.uniform(-1,1,2)) * rate)
        self.wanderdirection = newdir
        return self.seek(self.position + newdir + self.normalize(self.velocity) * forward, velocity)
    
    def containment(self, probes):
        force = np.zeros(2)
        for probe in probes:
            v = self.normalize(self.velocity)
            probepoint = self.position +  v * probe[0] + np.array([v[1], -1*v[0]]) * probe[1]
            for p in self.scene.particles:
                if type(p) is Wall:
                    wallvector = p.end - p.start
                    probevector = probepoint - p.start
                    projection = p.start + wallvector * np.dot(probevector, wallvector) / np.linalg.norm(wallvector)**2
                    bary = cartesian_to_barycentric(probepoint, self.position, p.start, p.end)
                    if bary[0] < 0 and bary[1] > 0 and bary[2] > 0:
                        force += (projection - self.position) -  (probepoint - self.position)
                        if np.linalg.norm(self.velocity) < .5:
                            self.enableinfo = True
        return force
    
    def lineofsight(self, target):
        return None
    
    def pathfollow(self, path, direction, cpredict, velocity):
        force = np.zeros(2)
        prediction = self.position + self.velocity * cpredict
        worldrecord = np.array([np.inf, np.inf])
        pathposition = np.array([np.inf, np.inf])
        pathlength = 0
        pathposlength = 0
        for i in range(len(path.points)-1):
            pathvector = np.array(path.points[i+1]) - np.array(path.points[i])
            pathlength += np.linalg.norm(pathvector)
            predictionvector = prediction - np.array(path.points[i])
            projection = np.array(path.points[i]) + pathvector * np.dot(predictionvector, pathvector) / np.linalg.norm(pathvector)**2
            projection = np.clip(projection, np.array([min(path.points[i][0],path.points[i+1][0]), min(path.points[i][1],path.points[i+1][1])]),\
                                 np.array([max(path.points[i][0],path.points[i+1][0]), max(path.points[i][1],path.points[i+1][1])]))
            positionvector = self.position - np.array(path.points[i])
            posprojection = np.array(path.points[i]) + pathvector * np.dot(positionvector, pathvector) / np.linalg.norm(pathvector)**2
            posprojection = np.clip(posprojection, np.array([min(path.points[i][0],path.points[i+1][0]), min(path.points[i][1],path.points[i+1][1])]),\
                                 np.array([max(path.points[i][0],path.points[i+1][0]), max(path.points[i][1],path.points[i+1][1])]))
            if np.linalg.norm(prediction - projection) < np.linalg.norm(prediction - worldrecord):
                worldrecord = projection
            if np.linalg.norm(posprojection - self.position) < np.linalg.norm(pathposition - self.position):
                pathposition = posprojection
                pathposlength = pathlength - np.linalg.norm(path.points[i+1] - pathposition)
        self.scene.delete(self.vectorid)
        if np.linalg.norm(prediction - worldrecord) > self.path.radius:
            self.displayarrows(path.pathlengthtoposition(pathposlength + direction) - self.position)
            force = self.seek(path.pathlengthtoposition(pathposlength + direction), velocity)
        return force
    
    def flowfieldfollow(self, field, cpredict):
        prediction = self.position + self.velocity * cpredict
        desired = field(prediction[0], prediction[1])
        return desired - self.velocity
    
    def collisionavoidance(self, approachlimit, timelimit):
        force = np.zeros(2)
        nearesttime = np.inf
        potentialcol = None
        for p in self.scene.particles:
            if not p == self:
                relvel = p.velocity - self.velocity
                proj = p.position + relvel * np.dot(self.position - p.position, relvel) / np.linalg.norm(relvel)**2
                nearest = np.linalg.norm(proj - self.position)
                time = np.dot(self.position - p.position, relvel) / np.linalg.norm(relvel) / np.linalg.norm(p.velocity)
                if time > 0 and nearest < approachlimit and time < nearesttime and time < timelimit:
                    nearesttime = time
                    potentialcol = p
        #self.scene.delete(self.vectorid)
        if potentialcol:
            predict0 = self.position + self.velocity * nearesttime
            predict1 = potentialcol.position + potentialcol.velocity * nearesttime
            force = self.normalize(predict0 - predict1)
            #self.displayarrows(potentialcol.position - self.position, fill="red")
        return force
    
    def seperation(self, radius, sweep):
        force = np.zeros(2)
        for p in self.scene.particles:
            if not p == self and not type(p) == Wall:
                distance = np.linalg.norm(p.position - self.position)
                dotproduct = np.dot(self.normalize(p.position - self.position), self.normalize(self.velocity))
                if distance < radius and dotproduct > (1-2*sweep) and not distance==0:
                    force += self.normalize(self.position - p.position) / (distance / radius)
        return force
    
    def cohesion(self, radius, sweep):
        force = np.zeros(2)
        num = 0
        avg = np.zeros(2)
        for p in self.scene.particles:
            if not p == self and not type(p) == Wall:
                distance = np.linalg.norm(p.position - self.position)
                dotproduct = np.dot(self.normalize(p.position - self.position), self.normalize(self.velocity))
                if distance < radius and dotproduct > (1-2*sweep) and not distance==0:
                    avg += p.position
                    num += 1
        if not num == 0:
            force = self.seek(avg/num, self.maxspeed)
        return force
    
    def alignment(self, radius, sweep):
        force = np.zeros(2)
        num = 0
        avg = np.zeros(2)
        for p in self.scene.particles:
            if not p == self and not type(p) == Wall:
                distance = np.linalg.norm(p.position - self.position)
                dotproduct = np.dot(self.normalize(p.position - self.position), self.normalize(self.velocity))
                if distance < radius and dotproduct > (1-2*sweep) and not distance==0:
                    avg += self.normalize(p.velocity)
                    num += 1
        if not num == 0:
            force = avg/num * self.maxspeed - self.velocity
        return force
    
    def leaderfollow(self, leader, offset, clearforward, clearside, slowing_distance, velocity):
        force = np.zeros(2)
        v = self.normalize(leader.velocity)
        a0 = leader.position + clearside * np.array([v[1], -v[0]])
        a1 = a0 + clearforward * v
        a2 = leader.position + clearside * np.array([-v[1], v[0]])
        a3 = a2 + clearforward * v
        if (not any([x<0 for x in cartesian_to_barycentric(self.position, a0, a1, a2)]) or not any([x<0 for x in cartesian_to_barycentric(self.position, a3, a1, a2)])):
            force += self.maxspeed * np.sign(np.dot(self.position - leader.position, np.array([v[1], -v[0]]))) * np.array([v[1], -v[0]]) - self.velocity
        else:
            force += self.arrive(leader.position - offset * v, slowing_distance, velocity)
        return force
    
    def flyingv(self, target, cpredict, slope):
        force = np.zeros(2)
        e0 = self.normalize(target.velocity)
        e1 = np.array([e0[1], -e0[0]])
        relativeposition = self.position - target.position
        relativecoords = np.array([np.dot(relativeposition, e1), np.dot(relativeposition, e0)])
        if relativecoords[1] < 0:
            relativevelocity = self.velocity - target.velocity
            vt = np.array([np.dot(relativevelocity, e1), np.dot(relativevelocity, e0)])
            predicted = relativecoords #+ cpredict * vt
            x = predicted[0]
            y = predicted[1]
            desired = 10*np.array([np.sign(x)*np.sign(y)*(y+np.abs(slope*x))-.1*x, (-np.abs(slope*x)-y)-.1*y])
            desired = desired / np.linalg.norm(relativecoords)
            rf = desired - vt
            force += rf[0]*e1 + rf[1]*e0
        return force
    
    def shadow(self, target, offset):
        force = np.zeros(2)
        dist = np.linalg.norm(self.position - target.position)
        if dist < offset:
            force += target.velocity - self.velocity
        else:
            force += self.pursue(target, self.maxspeed)
        return force
    
    def lineofsight(self, position):
        probepoint = position
        for p in self.scene.particles:
            if type(p) is Wall:
                wallvector = p.end - p.start
                probevector = probepoint - p.start
                projection = p.start + wallvector * np.dot(probevector, wallvector) / np.linalg.norm(wallvector)**2
                bary = cartesian_to_barycentric(probepoint, self.position, p.start, p.end)
                if bary[0] < 0 and bary[1] > 0 and bary[2] > 0:
                    return False
        return True
    
    
        
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
    
    def displayarrows(self, vector, **kw):
        self.scene.delete(self.vectorid)
        self.vectorid = self.scene.segment(self.position , self.position + vector, **kw)
        
    def repelborders(self):
        dx1 = (self.scene.winfo_width() - self.position[0]) / self.scene.winfo_width()
        dx2 = (0. - self.position[0]) / self.scene.winfo_width()
        dy1 = (self.scene.winfo_height() - self.position[1]) / self.scene.winfo_height()
        dy2 = (0. - self.position[1]) / self.scene.winfo_height()
        boundary1 = 1. / dx1**3 + 1. / dx2**3
        boundary2 = 1. / dy1**3 + 1. / dy2**3
        #self.displayinfo(np.array([-boundary1, -boundary2]))
        self.addforce(.000001 * np.array([-boundary1, -boundary2]))
        
    
def cartesian_to_barycentric(p, a0, a1, a2):
    detA = (a1[1] - a2[1]) * (a0[0] - a2[0]) + (a2[0] - a1[0]) * (a0[1] - a2[1])
    t0 = (a1[1] - a2[1]) * (p[0] - a2[0]) + (a2[0] - a1[0]) * (p[1] - a2[1])
    t0 /= detA
    t1 = (a2[1] - a0[1]) * (p[0] - a2[0]) + (a0[0] - a2[0]) * (p[1] - a2[1])
    t1 /= detA
    t2 = 1 - t0 - t1
    return [t0, t1, t2]    
    
class Wall(Particle):
    def __init__(self,scene, start, end, position, velocity=np.zeros(2), **kwargs):
        super().__init__(scene, position, velocity=velocity, **kwargs)
        self.start = np.array(start)
        self.end = np.array(end)
        
    def render(self, **kwargs):
        id = self.scene.segment(self.start, self.end, **kwargs)
        return id
        
class Path(Particle):
    def __init__(self, scene, points, radius, isloop=False, **kwargs):
        super().__init__(scene, [0,0], velocity=np.zeros(2), **kwargs)
        self.points = points
        self.radius = radius
        self.ids = None
        self.isloop = isloop
        self.hidden = False
        
    def render(self, **kwargs):
        ids = []
        for i in range(len(self.points)-1):
            ids.append(self.scene.segment(self.points[i], self.points[i+1],width=2*self.radius, stipple='gray12',capstyle="round", **kwargs))
            ids.append(self.scene.segment(self.points[i], self.points[i+1], **kwargs))
        return ids
    
    def pathlengthtoposition(self, length):
        walklength = 0
        result = np.zeros(2)
        for i in range(len(self.points)-1):
            v = np.array(self.points[i+1]) - np.array(self.points[i])
            walklength += np.linalg.norm(v)
            if walklength > length:
                result = np.array(self.points[i+1]) - self.normalize(v) * (walklength - length)
                break
        if length > walklength:
            if self.isloop:
                result = self.pathlengthtoposition(length - walklength)
            else:
                result = (length - walklength) * self.normalize(np.array(self.points[-1]) - np.array(self.points[-2]))
        return result
    
    def update(self):
        if self.ids == None and not self.hidden:
            self.ids = self.render(**self.kwargs)

class Seeker(Vehicle):
    def __init__(self, scene, position, velocity=np.zeros(2), **kwargs):
        super().__init__(scene, position, velocity=velocity, **kwargs)
        
    def update(self):
        force = np.array([0.,0.])
        force += self.arrive(np.array([400,400]), 100 , self.maxspeed)
        #force += self.flee(np.array([400,400]), self.maxspeed)
        self.displayinfo("seeking " + str(np.array([400,400])))
        self.update_refined_motion(force, wrapborder=True)
        
        
class Wanderer(Vehicle):
    def __init__(self, scene, position, velocity=np.zeros(2), **kwargs):
        super().__init__(scene, position, velocity=velocity, **kwargs)
        self.timer = 0
        self.maxspeed= 4.
        
    def update(self):
        force = np.array([0.,0.])
        force += self.wander(.05,1.1, self.maxspeed)
        force += self.containment([[80,0], [10,10], [10,-10]])
        #force += self.collisionavoidance(80, 15)
        self.update_refined_motion(force, wrapborder=True)
        self.timer -= self.scene.deltatime / 100.

        if self.timer < 0:
            self.scene.delete(self.textid)
        if self.enableinfo:
            self.displayinfo('FOLLOW ME THIS WAY!')
            self.enableinfo = False
            self.timer = 5.
            
class Pathfollower(Vehicle):
    def __init__(self, scene, position, velocity=np.zeros(2), **kwargs):
        super().__init__(scene, position, velocity=velocity, **kwargs)
        self.path = None
        self.maxspeed = 4.
        self.maximumforce = .7
    
    def update(self):
        force = np.array([0.,0.])
        force += self.seek(self.position + self.normalize(self.velocity) * self.maxspeed, self.maxspeed)
        force += self.pathfollow(self.path, 60, 10, self.maxspeed)
        self.update_refined_motion(force, wrapborder=True)
        
        
class Flowfollower(Vehicle):
    def __init__(self, scene, position, velocity=np.zeros(2), **kwargs):
        super().__init__(scene, position, velocity=velocity, **kwargs)
        self.field = None
        
    def update(self):
        force = np.array([0.,0.])
        force += self.flowfieldfollow(self.field, 10)
        self.update_refined_motion(force, wrapborder=True)
        
class Collisionavoider(Vehicle):
    def __init__(self, scene, position, velocity=np.zeros(2), **kwargs):
        super().__init__(scene, position, velocity=velocity, **kwargs)
        self.timer = 0
        self.maximumforce = .3
        self.maxspeed = 5
        
    def update(self):
        force = np.array([0.,0.])
        force += .5 * self.wander(.3,1.1, self.maxspeed * .8)
        force += 1 * self.seperation(40, 1)
        force += 1 * self.normalize(self.cohesion(50, 1))
        force += .7 * self.normalize(self.alignment(50, 1))
        force += self.containment([[100,0], [10,10], [10,-10]])
        self.update_refined_motion(force, wrapborder=True)
        self.timer -= self.scene.deltatime / 100.
        if self.timer < 0:
            self.scene.delete(self.textid)
        for p in self.scene.particles:
            if (not p == self) and np.linalg.norm(p.position - self.position) < 8:
                self.displayinfo('HEY WATCH IT!!')
                self.timer = 3.
    
class Follower(Vehicle):
    def __init__(self, scene, position, velocity=np.zeros(2), **kwargs):
        super().__init__(scene, position, velocity=velocity, **kwargs)
        self.target = None
        self.side = None
        self.maxspeed = 6
        self.maximumforce = .3
        self.offangle = 1-2*np.random.randint(0,2) * 45.
        
    def update(self):
        force = np.array([0.,0.])
        #force += 1.51 * self.leaderfollow(self.target, 30, 100, 25, 50, self.maxspeed)
        
        n = self.nearest(100, 1)
        if n:
            if n.target and not n.target==self:
                self.target = n.target
            else:
                self.target = n
        else:
            self.target = None
        if self.target:
            force += 20*self.flyingv(self.target, 40, 1.5)
            force += .5 * self.seperation(50, 1)
            force += .1 * self.normalize(self.cohesion(50, 1))
            force += .5 * self.normalize(self.alignment(50, 1))
        else:
            force += self.wander(.1,1.1, self.maxspeed*.8)
        #force += .5* self.arrive(self.target.position, 60, self.maxspeed)
        #force += self.collisionavoidance(60, 15)
        
        #force += self.containment([[80,0], [10,10], [10,-10]])
        self.update_refined_motion(force, wrapborder=True)
        
    def nearest(self, radius, sweep):
        best = np.inf
        neighbor = None
        for p in self.scene.particles:
            if not p == self:
                distance = np.linalg.norm(p.position - self.position)
                dotproduct = np.dot(self.normalize(p.position - self.position), self.normalize(self.velocity))
                if distance < radius and dotproduct > (1-2*sweep) and not distance==0 and distance < best:
                    best = distance
                    neighbor = p
        return neighbor
    
class Stalker(Vehicle):
    def __init__(self, scene, position, velocity=np.zeros(2), **kwargs):
        super().__init__(scene, position, velocity=velocity, **kwargs)
        self.target = None
        self.side = None
        self.maxspeed = 6
        self.maximumforce = .3
        self.timer = 0
        self.color = "black"
        self.arrowscolor = "orange"
        self.circlecolor = "red"
        self.neighbor = None
        self.debug = True
    
    def update(self):
        force = np.array([0.,0.])
        force += self.containment([[80,0], [10,10], [10,-10]])
        self.color = "black"
        self.circlecolor = "red"
        if self.target:
            if self.lineofsight(self.target.position):
                self.arrowscolor = "blue"
                if self.nearest(200,1)==None:
                    force += self.pursue(self.target, self.maxspeed)
                    self.color = "red"
                    self.circlecolor = "green"
                    if np.linalg.norm(self.position - self.target.position) < 10:
                        self.scene.particles.pop(self.scene.particles.index(self.target))
                        self.scene.delete(self.target.id)
                        self.scene.delete(self.target.textid)
                        self.scene.segment(self.target.position, self.target.position,width=30, stipple='gray25',capstyle="round", fill="red")
                        self.target = None
                        self.displayinfo('AAAEEAGHHH!')
                        self.timer = 10
                        for p in self.scene.particles:
                            if type(p)==Collisionavoider:
                                self.target = p
                                break
                else:
                    self.neighbor = self.nearest(200,1)
                    force += self.shadow(self.target, 100)
                    force +=  1 * self.seperation(30, 1)
            else:
                self.arrowscolor = "orange"
                force += 1 * self.wander(.4, 1.5, self.maxspeed)
                force +=  1 * self.seperation(30, 1)
        self.update_refined_motion(force, wrapborder=True)
        self.timer -= self.scene.deltatime / 100.
        if self.timer < 0:
            self.scene.delete(self.textid)
            
    def render(self, length, **kwargs):
        if self.id:
            for i in self.id:
                self.scene.delete(i)
        ids = []
        mag = np.sqrt(np.sum(self.velocity**2))
        if mag == 0:
            norm = np.array([1.,0.])
        else:
            norm = self.velocity / mag
        start = self.position - length / 2 * norm
        end = self.position + length / 2 * norm
        if self.debug and self.target:
            if not self.color == "red":
                self.color = "purple"
            self.displayarrows(self.target.position - self.position, dash=(3,4,3), fill=self.arrowscolor)
            ids.append(self.scene.circle(self.position, 200, dash=(20,5), outline=self.circlecolor))
            ids.append(self.scene.create_bitmap(self.target.position[0],self.target.position[1]-20, bitmap="warning", foreground="purple"))
            if self.neighbor:
                ids.append(self.scene.create_bitmap(self.neighbor.position[0],self.neighbor.position[1]-20, bitmap="questhead"))
        ids.append(self.scene.segment(start, end, arrow="last", fill=self.color, **kwargs))
        return ids
        
    def nearest(self, radius, sweep):
        best = np.inf
        neighbor = None
        for p in self.scene.particles:
            if not p == self and not p == self.target and not type(p)==Wall:
                distance = np.linalg.norm(p.position - self.position)
                dotproduct = np.dot(self.normalize(p.position - self.position), self.normalize(self.velocity))
                if distance < radius and dotproduct > (1-2*sweep) and not distance==0 and distance < best:
                    best = distance
                    neighbor = p
        return neighbor
    
    
class Waypoint(Particle):   
    def __init__(self, scene, position, velocity=np.zeros(2), **kwargs):
        super().__init__(scene, position, velocity=velocity, **kwargs)
        
def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
        return v
    return v / norm
    
def main():
   app = App()
   scene = Scene(app.mainframe, width=1000, height=800)
   scene.deltatime = 20
   wall = Wall(scene, [10,10], [1000,10] ,np.zeros(2))
   scene.addparticle(wall)
   wall = Wall(scene, [10,10], [10,800] ,np.zeros(2))
   scene.addparticle(wall)
   wall = Wall(scene, [1000,800], [1000,10] ,np.zeros(2))
   scene.addparticle(wall)
   wall = Wall(scene, [1000,800], [10,800] ,np.zeros(2))
   scene.addparticle(wall)
   for i in range(5):
       wall = Wall(scene, 1000 * np.random.rand(2), 800 * np.random.rand(2) ,np.zeros(2))
       scene.addparticle(wall)
   for i in range(1):
       p = Wanderer(scene, 200 * np.random.rand(2) + 100,velocity=np.array([0,10]), length=20)
       #scene.addparticle(p)
   path = Path(scene, [[100,50], [150,500], [100,600],[200,700],[800,600],[700,100],[500,500], [100,50]], 20, isloop=True)
   #scene.addparticle(path)
   for i in range(20):
       f = Pathfollower(scene,800 * np.random.rand(2), np.random.rand(2), length=20)
       f.path = path
       #scene.addparticle(f)
      
   field = lambda x,y: 5 * normalize(np.array([y**2, -x**2]))
   leader = Wanderer(scene, 200 * np.random.rand(2) + 100,velocity=np.array([0,10]), length=20, fill="blue")
   #scene.addparticle(leader)
   
   for i in range(10):
       p = Follower(scene, 800 * np.random.rand(2), np.random.rand(2)-np.array([.5,.5]), length=20)
       #p.target = leader
       #leader = p
       #scene.addparticle(p)
       p = Collisionavoider(scene, 600 * np.random.rand(2) + np.array([50,50]), 10 * (np.random.rand(2)-np.array([.5,.5])), length=20)
       scene.addparticle(p)
   stalker = Stalker(scene, 600 * np.random.rand(2) + np.array([50,50]), 10 * (np.random.rand(2)-np.array([.5,.5])), length=20)
   stalker.target = p
   stalker.debug = True
   scene.addparticle(stalker)
   ys = np.linspace(0,800, 10)
   xs = np.linspace(0,1000, 12)
   for y in ys:
       for x in xs:
           #line = scene.segment([x,y], np.array([x,y]) + 4 * field(x,y), fill="blue")
           pass
           
   app.mainloop()

if __name__ == '__main__':
    main()