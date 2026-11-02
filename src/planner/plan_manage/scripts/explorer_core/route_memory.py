"""Bounded route witnesses, lazy validation; no map, topology or timer owner."""
from collections import OrderedDict, defaultdict
import math
from threading import RLock
from .grid import bresenham
from .path_planning import adjacent_grid_path_is_valid


class RouteMemory(OrderedDict):
    """The existing route cache itself, with a small spatial sample index.

    Actual traversal is another witness, never a current motion certificate.
    Changes away from a requested route cost no work. A touched route is
    checked against the request's current map before any segment is reused.
    """
    def __init__(self, max_entries=48, max_cells=16000):
        super().__init__()
        self.max_entries, self.max_cells = max_entries, max_cells
        self.buckets = defaultdict(set)
        self.lock = RLock()
        self.motion = []
        self.hits = self.path_checks = self.motion_records = 0

    def _remove_index(self, key, path):
        for i in range(0,len(path),2):
            bucket = (path[i][0]//5,path[i][1]//5)
            self.buckets[bucket].discard((key,i))
            if not self.buckets[bucket]:del self.buckets[bucket]

    def __setitem__(self,key,path):
        with self.lock:
            if len(path)>self.max_cells:return
            self.pop(key,None)
            super().__setitem__(key,list(path))
            for i in range(0,len(path),2):
                self.buckets[(path[i][0]//5,path[i][1]//5)].add((key,i))
            while len(self)>self.max_entries or sum(map(len,self.values()))>self.max_cells:
                self.popitem(last=False)

    def pop(self,key,default=None):
        with self.lock:
            path=super().pop(key,None)
            if path is not None:self._remove_index(key,path)
            return default if path is None else path

    def popitem(self,last=True):
        with self.lock:
            key=next(reversed(self)) if last else next(iter(self))
            return key,self.pop(key)

    def clear(self):
        with self.lock:
            super().clear();self.buckets.clear();self.motion=[]

    def record_motion(self,cell):
        with self.lock:
            if self.motion and math.dist(self.motion[-1],cell)>5:
                self.motion=[] # reset/teleport is not a traversal
            if not self.motion or cell!=self.motion[-1]:self.motion.append(cell)
            if len(self.motion)>=12:
                path=[]
                for a,b in zip(self.motion,self.motion[1:]):
                    path.extend(bresenham(a,b)[:-1])
                path.append(cell)
                self[(path[0],path[-1])]=path
                self.motion_records+=1;self.motion=[cell]

    def connect(self,grid,start,goal,forbidden):
        with self.lock:
            def nearby(p):
                bx,by=p[0]//5,p[1]//5
                samples=set()
                for x in range(bx-1,bx+2):
                    for y in range(by-1,by+2):samples.update(self.buckets.get((x,y),()))
                return sorted((v for v in samples if v[0] in self and
                    math.dist(p,self[v[0]][v[1]])*grid.resolution<=1.),
                    key=lambda v:math.dist(p,self[v[0]][v[1]]))[:12]
            ends=nearby(goal)
            for key,i in nearby(start):
                for other,j in ends:
                    if key!=other:continue
                    path=self[key]
                    mid=list(path[i:j+1]) if i<=j else list(reversed(path[j:i+1]))
                    points=bresenham(start,mid[0])[:-1]+mid+bresenham(mid[-1],goal)[1:]
                    self.path_checks+=1
                    if adjacent_grid_path_is_valid(grid,points,set(),forbidden):
                        self.move_to_end(key);self.hits+=1
                        return points
            return None

    def stats(self):
        with self.lock:
            return dict(records=len(self),cells=sum(map(len,self.values())),hits=self.hits,
                        path_checks=self.path_checks,motion_records=self.motion_records)
