import numpy as np
import matplotlib.pyplot as plt


class Atmosphere:
    def __init__(self):
        self.altitude_axis = [0., 11000., 20000., 32000., 47000., 51000., 71000.]
        self.pressure_axis = [101325., 22632., 5475., 868., 110.9, 66.94, 3.96]
        self.density_axis = [1.225, .36391, .08803, .01322, .00143, .00086, .000064]
        self.press = self.pressure_axis[0]
        self.rho = self.density_axis[0]

    @property
    def pressure(self):
        return self.press

    @property
    def density(self):
        return self.rho

    def pressure_lookup(self, altitude):
        self.press = np.interp(altitude, self.altitude_axis, self.pressure_axis)
        return self.press

    def density_lookup(self, altitude):
        self.rho = np.interp(altitude, self.altitude_axis, self.density_axis)
        return self.rho

    def view_pressure_table(self):
        plt.title('Atmospheric Pressure (N/m^2) vs. Altitude (m)')
        plt.plot(self.altitude_axis, self.pressure_axis, 'b')
        plt.xlabel('Altitude (m)')
        plt.ylabel('Atmospheric Pressure (N/m^2)')
        plt.grid()
        plt.show()

    def view_density_table(self):
        plt.title('Atmospheric Density (kg/m^3) vs. Altitude (m)')
        plt.plot(self.altitude_axis, self.density_axis, 'b')
        plt.xlabel('Altitude (m)')
        plt.ylabel('Atmospheric Density (kg/m^3)')
        plt.grid()
        plt.show()
