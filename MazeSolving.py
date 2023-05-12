# -*- coding: utf-8 -*-

from Automatons import *
import colorsys
import random
from PIL import Image, ImageDraw


colorbydistance = False
colorbyzone = False
zones = 3
scale = 32
width, height = 64, 32
x, y = random.randint(0, width-1), random.randint(0, height-1)
remaining = width * height - 1

grid = [[set() for y in range(height)] for x in range(width)]

directions = ["N","S","E","W"]
dx = { "N": 0, "S": 0, "E": -1, "W": 1}
dy = { "N": 1, "S": -1, "E": 0, "W": 0}
opp = { "N": "S", "S": "N", "E": "W", "W": "E"}

while remaining > 0:
    random.shuffle(directions)
    drc = directions[0]
    nx, ny = x + dx[drc], y + dy[drc]
    if nx < 0 or ny < 0 or nx >= width or ny >= height:
        continue
    if len(grid[nx][ny]) == 0:
        grid[x][y].add(drc)
        grid[nx][ny].add(opp[drc])
        remaining -= 1
    x, y = nx, ny 


class Queue:
    def __init__(self):
        self.items = []
    def is_empty(self):
        return self.items == []
    def enqueue(self, item):
        self.items.append(item)
    def dequeue(self):
        return self.items.pop(0)
    def size(self):
        return len(self.items)
    
class Stack:
    def __init__(self):
        self.items = []
    def push(self, item):
        self.items.append(item)
    def pop(self):
        return self.items.pop()
    def is_empty(self):
        return self.items == []
    def peek(self):
        return self.items[-1]
    def size(self):
        return len(self.items)
        
class Node():
    def __init__(self, v=(0,0)):
        self.parent = None
        self.v = v
        self.distance = 0
        self.zone = 0
        self.children = []
        
    def branchsize(self):
        size = 1
        for child in self.children:
            size += child.branchsize()
        return size
    
    def zonebranch(self, zone):
        self.zone = zone
        for child in self.children:
            child.zonebranch(zone)
    
    def balancedpartition(self):
        if self.children == []:
            return None
        Q = Queue()
        Q.enqueue(self)
        bestpivot = self
        best = self.branchsize()
        while not Q.is_empty():
            root = Q.dequeue()
            for i in range(len(root.children)):
                pivot = root.children.pop(i)
                Q.enqueue(pivot)
                if abs(pivot.branchsize() - self.branchsize()) <= 1:
                    bestpivot = pivot
                    best = abs(pivot.branchsize() - self.branchsize())
                    root.children.insert(i, pivot)
                    break
                if abs(pivot.branchsize() - self.branchsize()) < best:
                    bestpivot = pivot
                    best = abs(pivot.branchsize() - self.branchsize())
                root.children.insert(i, pivot)
        bestpivot.parent.children.remove(bestpivot)
        bestpivot.parent = None
        return self, bestpivot
        

class MazeSolver(Vehicle):
    def __init__(self, scene, grid, root, position, scale, goal=(width-1, height-1), velocity=np.zeros(2), **kwargs):
        super().__init__(scene, position, velocity=velocity, **kwargs)
        self.path = None
        self.maxspeed = 8 * scale / 50
        self.maximumforce = 1.3 * scale / 50
        self.s = scale
        self.goal = (width-1, height-1)
        self.grid = grid
        self.S = Stack()
        self.explored = set()
        self.S.push(root)
        self.explored.add(root)
        self.timer = 0
        self.exploring = True
        self.hamber = False
        self.sayhamber = False
    
    def update(self):
        old_pos = self.position
        force = np.array([0.,0.])
        v = self.S.peek()
        target = self.s * np.array([v[0]+1, v[1]+1])
        if v == self.goal:
            target = np.array([width * scale, (height+.5) * scale])
        force += self.arrive(target, .5 * self.s, self.maxspeed)
        dist = np.linalg.norm(target - self.position)
        if dist < (.5 * self.s):
            self.iterateDFS2()
        self.update_refined_motion(force, wrapborder=False)
        self.scene.segment(old_pos, self.position, fill="blue")
        
        if self.timer < 0:
            self.scene.delete(self.textid)
            
        
    def iterateDFS2(self):
        v = self.S.pop()
        if v == self.goal:
            self.S.push(v)
            if self.sayhamber:
                self.displayinfo('"MEIN HAMBER"',font="bold")
            return v
        self.exploring = False
        for d in self.grid[v[0]][v[1]]:
            w = (v[0] + dx[d], v[1] + dy[d])
            if w not in self.explored:
                self.S.push(v)
                self.explored.add(w)
                self.S.push(w)
                self.exploring = True
                self.hamber = False
                break
        if not self.exploring and not self.hamber and self.sayhamber:
            self.displayinfo('"HAMBER"', font="bold")
            self.timer = 3.
            self.hamber = True

            
    def render(self, rscale, **kwargs):
        ids = []
        r = self.s * rscale
        mag = np.sqrt(np.sum(self.velocity**2))
        if mag == 0:
            norm = np.array([1.,0.])
        else:
            norm = self.velocity / mag
        side = np.array([norm[1], -norm[0]])
        start = self.position + r * side
        start2 = self.position - r * side
        end = self.position + 2 * r * norm
        end2 = self.position - 2.5 * r * norm
        ids.append(self.scene.segment(start, end, **kwargs, width=r*.6))
        ids.append(self.scene.segment(start2, end, **kwargs, width=r*.6))
        ids.append(self.scene.segment(self.position, end2, **kwargs, width=r*.4, fill="pink"))
        ids.append(self.scene.circle(self.position, r, fill="black"))
        return ids
        
        
class BaseMazeSolver:
    def __init__(self, grid, root, goal=(width-1, height-1)):
        self.maxdistance = 0
        self.root = root
        self.s = scale
        self.Q = Queue()
        self.S = Stack()
        self.M = Stack()
        self.distance = [[0 for y in range(height)] for x in range(width)]
        self.zones = [[0 for y in range(height)] for x in range(width)]
        self.grid = grid
        self.goal = goal
        self.explored = set()
        self.explored.add(root)
        self.Q.enqueue(root)
        self.S.push(root)
        self.M.push(root)
        self.explored.add(root)
        self.nodegrid = [[Node((x,y)) for y in range(height)] for x in range(width)]
        self.nodes = set()
        self.tree = None
        
        
    def createzones(self, num):
        self.buildtree()
        Q = Queue()
        Q.enqueue(self.tree)
        while Q.size() < num:
            tree = Q.dequeue()
            t1, t2 = tree.balancedpartition()
            Q.enqueue(t1)
            Q.enqueue(t2)
            
        zone = 0
        while not Q.is_empty():
            tree = Q.dequeue()
            tree.zonebranch(zone)
            zone += 1
        
        for i in range(width):
            for j in range(height):
                self.zones[i][j] = self.nodegrid[i][j].zone
        
        
    def buildtree(self):
        self.explored = set()
        self.explored.add(self.root)
        self.Q = Queue()
        self.Q.enqueue(self.root)
        while not self.Q.is_empty():
            v = self.Q.dequeue()
            for d in self.grid[v[0]][v[1]]:
                w = (v[0] + dx[d], v[1] + dy[d])
                if w not in self.explored:
                    self.explored.add(w)
                    self.Q.enqueue(w)
                    self.nodegrid[v[0]][v[1]].children.append(self.nodegrid[w[0]][w[1]])
                    self.nodegrid[w[0]][w[1]].parent = self.nodegrid[v[0]][v[1]]
                    self.nodes.add(self.nodegrid[w[0]][w[1]])
        self.tree = self.nodegrid[0][0]
        self.nodes.add(self.nodegrid[0][0])
                    
        
    def finddistances(self):
        self.M = Stack()
        self.explored = set()
        self.explored.add(self.root)
        self.M.push(self.root)
        while len(self.explored) < (width * height):
            v = self.M.pop()
            for d in self.grid[v[0]][v[1]]:
                w = (v[0] + dx[d], v[1] + dy[d])
                if w not in self.explored:
                    self.M.push(v)
                    self.explored.add(w)
                    self.M.push(w)
                    self.distance[w[0]][w[1]] = self.M.size()
                    if self.M.size() > self.maxdistance:
                        self.maxdistance = self.M.size()
                    break
            
    def iterateBFS(self):
        v = self.Q.dequeue()
        if v == self.goal:
            return v
        for d in self.grid[v[0]][v[1]]:
            w = (v[0] + dx[d], v[1] + dy[d])
            if w not in self.explored:
                self.explored.add(w)
                self.Q.enqueue(w)
                
    def iterateDFS(self):
        v = self.S.pop()
        if v not in self.explored:
            self.explored.add(v)
            if v == self.goal:
                return v
            for d in self.grid[v[0]][v[1]]:
                w = (v[0] + dx[d], v[1] + dy[d])
                self.S.push(w)
    
    def iterateDFS2(self):
        v = self.M.pop()
        if v == self.goal:
            self.M.push(v)
            return v
        for d in self.grid[v[0]][v[1]]:
            w = (v[0] + dx[d], v[1] + dy[d])
            if w not in self.explored:
                self.M.push(v)
                self.explored.add(w)
                self.M.push(w)
                break
            


def main():
    solver = BaseMazeSolver(grid, (0,0))
    solver.finddistances()
    solver.createzones(zones)
    app = App()
    scene = Scene(app.mainframe, width=(1+width)*scale, height=(1+height)*scale, bg="grey")
    colors = []
    

    for i in range(zones):
        colors.append((random.randint(0,255), random.randint(0,255), random.randint(0,255)))
    for i in range(width):
        for j in range(height):
            for d in grid[i][j]:
                start = [(1+i) * scale, (1+j) * scale]
                end = [((1+i) + dx[d]) * scale, ((1+j) + dy[d]) * scale]
                if colorbydistance:
                    rgb = colorsys.hls_to_rgb((solver.distance[i][j] / solver.maxdistance) * .8, .5, 1)
                    r, g, b = [int(x * 255) for x in rgb]
                    color = '#%02x%02x%02x' % (r,g,b)
                elif colorbyzone:
                    rgb = colorsys.hls_to_rgb((solver.zones[i][j] / zones) * .8, .5, 1)
                    r, g, b = [int(x * 255) for x in rgb]
                    color = '#%02x%02x%02x' % colors[solver.zones[i][j]]
                else:
                    color = "white"
                scene.segment(start, end, width=scale*.6, fill=color, capstyle="projecting")

    scene.circle([scale, scale/2], scale*.3,  fill="green")
    scene.circle([width * scale, (height+.5) * scale], scale*.3, fill="yellow")
    rat = MazeSolver(scene, grid, (0,0), [scale, scale/2], scale, rscale=.1)
    scene.addparticle(rat)  
    app.mainloop()

if __name__ == '__main__':
    main()
