import numpy as np
import matplotlib.pyplot as plt
import math


class RK4:
    def __init__(self, **init_kwargs):
        init_pos_x = init_kwargs['pos_x']
        init_vel_x = init_kwargs['vel_x']
        init_pos_y = init_kwargs['pos_y']
        init_vel_y = init_kwargs['vel_y']
        self.u = np.zeros((1, 4))
        self.u[0] = np.array([init_pos_x, init_vel_x, init_pos_y, init_vel_y])
        self.t = 0.
        self.hold_t = [self.t]
        self.count = 0

    @property
    def time(self):
        return round(self.t, 1)

    def func(self, states, t, params=None):
        if params:
            G = params['G']
            m1 = params['m1']
            m2 = params['m2']
            r = params['r']
            earth_x = params['earth pos'][0]
            earth_y = params['earth pos'][1]
            Ft = params['thrust']
            pitch_rad = params['pitch'] * 0.0174533
            pressure = params['atm. pressure']
            Cd = params['Cd']
            A = params['A']
            rho = params['rho']
        else:
            G, m1, m2, r, earth_x, earth_y, Ft, pitch_rad, pressure, Cd, A = 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.
            rho = 0.

        pos_x, vel_x, pos_y, vel_y = states

        earth_accel_dir = np.array([(pos_x-earth_x), (pos_y-earth_y)])
        earth_accel_dir_unit = earth_accel_dir/np.linalg.norm(earth_accel_dir)

        Fg = G * ((m1*m2)/r**2)
        Fg *= -earth_accel_dir_unit

        vel_dir_unit = np.array([np.sin(pitch_rad), np.cos(pitch_rad)])
        Ft *= vel_dir_unit

        tot_vel = np.linalg.norm([vel_x, vel_y])
        dyn_pressure = ((rho * tot_vel ** 2) / 2)
        Fd = Cd * A * dyn_pressure
        Fd *= vel_dir_unit

        accel_x = Fg[0]/m1 + Ft[0]/m1 - Fd[0]/m1
        accel_y = Ft[1]/m1 + Fg[1]/m1 - Fd[1]/m1

        return np.array([vel_x, accel_x, vel_y, accel_y])

    def rk4(self, h, params=None):
        self.t += h
        self.hold_t.append(self.t)

        k1 = h * self.func(self.u[self.count], self.t, params)
        k2 = h * self.func(self.u[self.count] + 0.5 * k1, self.t + 0.5*h, params)
        k3 = h * self.func(self.u[self.count] + 0.5 * k2, self.t + 0.5*h, params)
        k4 = h * self.func(self.u[self.count] + k3, self.t + 0.5*h, params)
        next_states = ([self.u[self.count] + (k1 + 2*(k2 + k3) + k4) / 6])
        self.u = np.vstack([self.u, next_states])

        self.count += 1

        return self.u, self.t

    def solve_step(self, dt, params=None):
        u, t = self.rk4(dt, params)
        # print('TIME: ', round(t, 3), 'STATES: ', u[-1])
        return u[-1]

    def plot_results(self, e_radius, crash=False):
        plt.title('X Position vs. Y Position')
        plt.plot(self.u[:, 0], self.u[:, 2])
        plt.plot(self.u[0, 0], self.u[0, 2], color='g', marker='o')

        if crash:
            plt.plot(self.u[-1, 0], self.u[-1, 2], color='r', marker='x')

        plt.grid()
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')

        # plt.xlim(-1E4, 3.5E5)
        # plt.ylim(.625E7, .66E7)

        plt.xlim(-30E6, 30E6)
        plt.ylim(-30E6, 30E6)

        earth = plt.Circle((0., 0.), e_radius, alpha=.75, color='b', fill=True)
        atmosphere = plt.Circle((0., 0.), e_radius+80E3, alpha=.2, color='g', fill=True)
        fig = plt.gcf()
        ax = fig.gca()
        ax.add_artist(earth)
        ax.add_artist(atmosphere)

        plt.show()

