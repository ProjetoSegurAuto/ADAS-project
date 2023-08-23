# Union Find by Felipe Santos

import numpy as np

class Car:
    def __init__(self, idx, X, GAP):
        self.idx = idx
        self.X = X
        self.GAP = GAP

class DSU:
    def __init__(self):
        self.MAXN = 110
        
        self.ds = np.arange(self.MAXN, dtype=int)
        self.size = np.zeros(self.MAXN, dtype=int)

    def dsBuild(self):
        for i in range(self.MAXN):
            self.ds[i] = i
            self.size[i] = 1
    
    def dsFind(self, car: int) -> int:
        if(car != self.ds[car]) :
            return self.dsFind(self.ds[car])
        else:
            return car
        
    def dsUnion(self, car_u: Car, car_v: Car):
        u = self.dsFind(car_u.idx)
        v = self.dsFind(car_v.idx)
        
        cood_u = car_u.X
        cood_v = car_v.X
        
        #validação lógica parcial(distância geografica). Obs: neste ponto eu já sei que existe a condição para Platoon
        if(cood_u < cood_v):
            u, v = v, u

        self.ds[v] = u
        self.size[u] += self.size[v]

    #Pensar melhor nesse algoritmo, pois só tá válido na tail
    def dsLeave(self, car_u : Car):
        
        # most_dist = -1
        # new_platoon_father = -1

        # for i in range(self.length):
        #     if(i != car_u.idx and self.dsFind(car_u.idx) == car_u.idx):
        #         if(most_dist < cars[i].X):
        #             most_dist = cars[i].X
        #             new_platoon_father = i

        # if(new_platoon_father != -1):
        #     for i in range(self.length):
        #         if(self.dsFind(i) == self.dsFind(car_u.idx)):
        #             self.ds[i] = new_platoon_father
        
        self.size[self.dsFind(car_u.idx)] = self.size[self.dsFind(car_u.idx)] - 1
        self.ds[car_u.idx] = car_u.idx
        self.size[car_u.idx] = 1

    def getSize(self, u: int) -> int:
        u = self.dsFind(u)
        return self.size[u]
