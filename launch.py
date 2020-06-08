from solver import *
from planet import *
from spacecraft import *
from general_info import *
from atmosphere import *


def check_crash(vehicle, planet):
    delt_x = (vehicle.position[0] - planet.position[0])
    delt_y = (vehicle.position[1] - planet.position[1])
    rng = np.sqrt(delt_x ** 2 + delt_y ** 2)

    if rng <= planet.radius:
        return True

    return False


atmosphere = Atmosphere()

planet_pos = np.array([0., 0.])
planet_vel = np.zeros(2)
planet_mass = Earth['Mass']
planet_radius = Earth['Radius']
planet = Planet(planet_pos, planet_vel, planet_mass, planet_radius)

spacecraft_pos = np.array([0., planet.radius+5.])
spacecraft_vel = np.array([0., 0.])
spacecraft_mass = Falcon_9['Mass']
spacecraft = SpaceCraft(spacecraft_pos, spacecraft_vel, spacecraft_mass)

pos = spacecraft_pos
vel = spacecraft_vel

solver = RK4(pos_x=spacecraft_pos[0], vel_x=spacecraft_vel[0], pos_y=spacecraft_pos[1], vel_y=spacecraft_vel[1])

params = {'G': 6.67259E-11}

start_burn_counter = False
cool_down = False
burn_timer = 0.
cooldown_timer = 0.
crash = False
end_time_min = 450.
end_time_sec = end_time_min * 60.
vehicle_pitch = 0.
while solver.time <= end_time_sec:
    DT = 2.5

    engine_cmd = 7.5E6
    rel_alt = spacecraft.polar_radius - planet.radius
    params['m1'] = spacecraft.mass
    params['m2'] = planet.mass
    params['r'] = np.abs(np.linalg.norm(spacecraft.position-planet.position))
    params['earth pos'] = planet.position
    params['atm. pressure'] = atmosphere.pressure_lookup(rel_alt)
    params['Cd'] = spacecraft.drag_coeff
    params['A'] = spacecraft.area
    params['rho'] = atmosphere.density_lookup(rel_alt)

    if solver.time >= 100.:
        vehicle_pitch = 30.

    if solver.time >= 1200.:
        engine_cmd = 0.

    spacecraft.update(pos, vel, engine_cmd, vehicle_pitch)

    params['thrust'] = spacecraft.thrust
    params['pitch'] = spacecraft.pitch_angle

    pos_x, vel_x, pos_y, vel_y = solver.solve_step(DT, params)

    pos = np.array([pos_x, pos_y])
    vel = np.array([vel_x, vel_y])

    # print('REL ALTITUDE: ', rel_alt)
    print('TIME: ', solver.time, 'VELOCITY: ', spacecraft.vel_mag)

    if check_crash(spacecraft, planet):
        print('CRASHED AT TIME: ', solver.time)
        crash = True
        break

    # print(solver.time, ' ', spacecraft.polar_radius-planet.radius)

solver.plot_results(planet.radius, crash)

