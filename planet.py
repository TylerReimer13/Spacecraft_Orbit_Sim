import numpy as np


class Planet:
    def __init__(self, pos, vel, m, r):
        self.pos = pos
        self.vel = vel
        self.m = m
        self.r = r

    @property
    def position(self):
        return self.pos.copy()

    @property
    def mass(self):
        return self.m

    @property
    def radius(self):
        return self.r

