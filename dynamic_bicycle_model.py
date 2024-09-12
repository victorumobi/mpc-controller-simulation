import numpy as np

class DynamicBicycleModel:
    def __init__(self, Lf, Lr, max_steer, max_accel, vehicle_mass, cornering_stiffness):
        self.Lf = Lf
        self.Lr = Lr
        self.max_steer = max_steer
        self.max_accel = max_accel
        self.vehicle_mass = vehicle_mass
        self.cornering_stiffness = cornering_stiffness
        self.gravity = 9.81

    def tire_forces(self, v, delta, slip_angle):
        # Pacejka tire model (simplified version for lateral forces)
        lateral_force = self.cornering_stiffness * slip_angle
        return lateral_force

    def dynamics(self, state, control_input):
        x, y, v, theta, omega = state  # omega is yaw rate
        a, delta = control_input

        # Limit control inputs
        a = np.clip(a, -self.max_accel, self.max_accel)
        delta = np.clip(delta, -self.max_steer, self.max_steer)

        # Calculate slip angles
        slip_angle_front = np.arctan((v + self.Lf * omega) / v) - delta
        slip_angle_rear = np.arctan((v - self.Lr * omega) / v)

        # Tire forces
        Ff = self.tire_forces(v, delta, slip_angle_front)
        Fr = self.tire_forces(v, 0, slip_angle_rear)

        # Update state
        x_dot = v * np.cos(theta) - omega * self.Lf
        y_dot = v * np.sin(theta) + omega * self.Lr
        v_dot = a
        theta_dot = omega
        omega_dot = (Ff * self.Lf - Fr * self.Lr) / self.vehicle_mass

        return np.array([x_dot, y_dot, v_dot, theta_dot, omega_dot])

    def update(self, state, control_input, dt):
        state_dot = self.dynamics(state, control_input)
        new_state = state + state_dot * dt
        return new_state
