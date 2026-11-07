"""Replaceable planar dynamics for the provisional P2 scenario, in SI units."""

from dataclasses import dataclass

import numpy as np


G0 = 9.80665
ATMOSPHERE_RADIUS = 6_356_766.0
AIR_GAS_CONSTANT = 287.05287
LAYER_HEIGHT = np.array([0, 11_000, 20_000, 32_000, 47_000, 51_000, 71_000])
LAPSE_RATE = np.array([-0.0065, 0.0, 0.001, 0.0028, 0.0, -0.0028, -0.002])
BASE_TEMPERATURE = np.array([288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65])
BASE_PRESSURE = np.array([101325, 22632.064, 5474.889, 868.0187, 110.9063, 66.93887, 3.956420])


@dataclass(frozen=True)
class Scenario:
    """Problem data and explicit engineering assumptions used by P2."""

    separation_altitude_m: float
    separation_speed_m_s: float
    dry_mass_kg: float
    flight_path_angle_deg: float
    effective_drag_area_m2: float
    specific_impulse_s: float
    min_thrust_n: float
    max_thrust_n: float
    max_thrust_tilt_deg: float
    touchdown_thrust_tilt_deg: float
    max_powered_tilt_rate_deg_s: float
    touchdown_altitude_m: float
    touchdown_vertical_speed_m_s: float
    reserve_kg: float
    unusable_kg: float
    oxidizer_to_fuel_ratio: float
    dynamic_pressure_limit_pa: float
    proper_acceleration_limit_g: float

    @property
    def minimum_mass(self):
        return self.dry_mass_kg + self.reserve_kg + self.unusable_kg

    def initial_state(self, mass):
        angle = np.deg2rad(self.flight_path_angle_deg)
        speed = self.separation_speed_m_s
        return np.array([0, self.separation_altitude_m, speed * np.cos(angle),
                         speed * np.sin(angle), mass])


def density(altitude):
    """Lower-atmosphere COESA model, valid through 86 km geometric altitude."""
    # Clip trial states only for evaluating the formula. Altitude bounds are
    # imposed separately in the optimization and checked in the replay.
    altitude = np.clip(altitude, 0, 86_000)
    geopotential = ATMOSPHERE_RADIUS * altitude / (ATMOSPHERE_RADIUS + altitude)
    layer = np.searchsorted(LAYER_HEIGHT, geopotential, side="right") - 1
    delta = geopotential - LAYER_HEIGHT[layer]
    lapse = LAPSE_RATE[layer]
    base_temperature = BASE_TEMPERATURE[layer]
    temperature = base_temperature + lapse * delta
    exponent = np.where(
        lapse == 0,
        -G0 * delta / (AIR_GAS_CONSTANT * base_temperature),
        -G0 * np.log(temperature / base_temperature)
        / (AIR_GAS_CONSTANT * np.where(lapse == 0, 1, lapse)),
    )
    return BASE_PRESSURE[layer] * np.exp(exponent) / (AIR_GAS_CONSTANT * temperature)


def forces(scenario, state, control):
    """Return dynamic pressure and the total nongravitational force."""
    velocity = state[..., 2:4]
    speed = np.linalg.norm(velocity, axis=-1)
    rho = density(state[..., 1])
    pressure = 0.5 * rho * speed**2
    drag = (-0.5 * rho * scenario.effective_drag_area_m2 * speed)[..., None] * velocity
    thrust, tilt = control[..., 0], control[..., 1]
    thrust_vector = thrust[..., None] * np.stack((np.sin(tilt), np.cos(tilt)), axis=-1)
    return pressure, thrust_vector + drag


def dynamics(scenario, state, control):
    """Derivative of [x, altitude, vx, vz, mass] under [thrust, tilt]."""
    _, force = forces(scenario, state, control)
    acceleration = force / state[..., 4, None] - np.array([0, G0])
    mass_rate = -control[..., :1] / (G0 * scenario.specific_impulse_s)
    return np.concatenate((state[..., 2:4], acceleration, mass_rate), axis=-1)


def flight_limits(scenario, states, controls):
    """Positive margins in the order altitude, mass, dynamic pressure, load."""
    pressure, force = forces(scenario, states, controls)
    load = np.linalg.norm(force, axis=-1) / (states[..., 4] * G0)
    return np.stack((
        (states[..., 1] - scenario.touchdown_altitude_m) / 70_000,
        (86_000 - states[..., 1]) / 70_000,
        (states[..., 4] - scenario.minimum_mass) / scenario.dry_mass_kg,
        1 - pressure / scenario.dynamic_pressure_limit_pa,
        1 - load / scenario.proper_acceleration_limit_g,
    ), axis=-1)
