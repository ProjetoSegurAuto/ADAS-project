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
        
        self.__ds = np.arange(self.MAXN, dtype=int)
        self.__size = np.zeros(self.MAXN, dtype=int)
        self.__set_of_car = [Car(-1,-1,-1)] * self.MAXN

    def dsBuild(self):
        for i in range(self.MAXN):
            self.__ds[i] = i
            self.__size[i] = 1
    
    def dsFind(self, car: int) -> int:
        if(car != self.__ds[car]) :
            return self.dsFind(self.__ds[car])
        else:
            return car
        
    def dsUnion(self, car_u: Car, car_v: Car):
        u = self.dsFind(car_u.idx)
        v = self.dsFind(car_v.idx)
        
        self.__set_of_car[car_u.idx] = car_u
        self.__set_of_car[car_v.idx] = car_v
        
        cood_u = self.__set_of_car[u].X
        cood_v = self.__set_of_car[v].X
        
        #validação lógica parcial(distância geografica). Obs: neste ponto eu já sei que existe a condição para Platoon
        if(cood_u < cood_v):
            u, v = v, u

        self.__ds[v] = u
        self.__size[u] += self.__size[v]

    def dsLeave(self, car_u : Car):
        #subconjunto de componente conexo
        subset = list()
        for i in range(self.MAXN):
            if(self.dsFind(i) == self.dsFind(car_u.idx)):
                subset.append(i)

        #rebuildar
        for i in subset:
            self.__ds[i] = i
            self.__size[i] = 1
        self[car_u.idx] = car_u.idx
        self.__size[car_u.idx] = 1
        
        #união
        for i in range(1, self.MAXN):
            self.dsUnion(self.__set_of_car[i-1], self.__set_of_car[i])

    def getSize(self, u: int) -> int:
        u = self.dsFind(u)
        return self.__size[u]
