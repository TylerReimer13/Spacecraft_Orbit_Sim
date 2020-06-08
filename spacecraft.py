import numpy as np
import math


class SpaceCraft:
    def __init__(self, pos, vel, m):
        self.pos = pos
        self.vel = vel
        self.accel = np.zeros(2)
        self.m = m
        self.fuel = 100.
        self.thrust_force = 0.
        self.pitch = 0.
        self.cd = .65
        self.A = 50.

    @property
    def position(self):
        return self.pos.copy()

    @property
    def velocity(self):
        return self.vel.copy()

    @property
    def vel_mag(self):
        return np.linalg.norm(self.velocity)

    @property
    def mass(self):
        return self.m

    @property
    def thrust(self):
        return self.thrust_force

    @property
    def pitch_angle(self):
        return self.pitch

    @property
    def polar_angle(self):
        return math.atan2(self.position[1], self.position[0])

    @property
    def polar_radius(self):
        return math.sqrt((self.position[0]**2) + (self.position[1]**2))

    @property
    def drag_coeff(self):
        return self.cd

    @property
    def area(self):
        return self.A

    def pitch_vehicle(self, degrees):
        self.pitch = degrees

    def actuate_thruster(self, thrust_force):
        self.fuel -= .1
        self.thrust_force = thrust_force

    def update(self, pos, vel, engine, pitch):
        self.vel = vel
        self.pos = pos
        self.actuate_thruster(engine)

        if pitch:
            self.pitch_vehicle(pitch)


class Engine:
    def __init__(self, area):
        self.nozzle_area = area
