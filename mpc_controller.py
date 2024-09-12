from casadi import *
import numpy as np

def mpc_controller(state, path, obstacles, N=10, dt=0.1):
    # Define the optimization variables
    x = SX.sym('x', N)
    y = SX.sym('y', N)
    v = SX.sym('v', N)
    theta = SX.sym('theta', N)
    a = SX.sym('a', N-1)
    delta = SX.sym('delta', N-1)

    # Define the cost function
    cost = 0
    constraints = []

    for i in range(N-1):
        # State updates based on the dynamic model
        x_dot = v[i] * cos(theta[i])
        y_dot = v[i] * sin(theta[i])

        # Objective: Minimize path tracking error and control effort
        path_error = (x[i] - path[i][0])**2 + (y[i] - path[i][1])**2
        control_effort = a[i]**2 + delta[i]**2

        # Add obstacle avoidance cost
        for obs in obstacles:
            obs_dist = (x[i] - obs[0])**2 + (y[i] - obs[1])**2
            cost += 100 / obs_dist  # High penalty for getting too close

        cost += path_error + 0.1 * control_effort

        # Constraints for dynamics, avoiding infeasible maneuvers
        constraints.append(x[i+1] == x[i] + x_dot * dt)
        constraints.append(y[i+1] == y[i] + y_dot * dt)
        constraints.append(v[i+1] == v[i] + a[i] * dt)
        constraints.append(theta[i+1] == theta[i] + delta[i] * dt)

    # Setup solver
    opt_variables = vertcat(x, y, v, theta, a, delta)
    nlp = {'x': opt_variables, 'f': cost, 'g': vertcat(*constraints)}
    solver = nlpsol('solver', 'ipopt', nlp)

    # Solve
    solution = solver(x0=np.zeros(N*6), lbx=-np.inf, ubx=np.inf)
    return solution['x'][0:2]  # Return control input
