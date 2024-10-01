import casadi as ca
import numpy as np
# import matplotlib.pyplot as plt

class MPC:
    def __init__(self, N, dt, kappa_coeffs, delta_max_degree=30):
        # MPC parameters
        self.N = N
        self.dt = dt
        self.L = 2.5

        # Vehicle parameters
        self.NX = 4
        self.NU = 2

        # Control limits
        self.delta_max = np.deg2rad(delta_max_degree)
        self.delta_min = -self.delta_max
        self.a_max = 5.0    
        self.a_min = -5.0
        self.v_max = 15.0
        self.v_min = 0.0

        # Weighting matrices
        self.Q = ca.diag(np.array([10.0, 10.0, 1.0, 0.0]))
        self.R = ca.diag(np.array([0.5, 0.5]))
        self.Rd = ca.diag(np.array([5.0,5.0]))

        # self.s_vals = s_vals
        # self.kappa_ref = kappa_ref
        self.kappa_coeffs = kappa_coeffs

        # self.kappa_fun = ca.interpolant('kappa', 'linear', [s_vals], kappa_ref)
        self.kappa_coeffs = kappa_coeffs
        # Initialize the sovler
        self._init_solver()


    def _init_solver(self):
        # Create symbolic variables
        d = ca.SX.sym('d')          # Lateral deviation
        psi = ca.SX.sym('psi')      # Orientation error
        v = ca.SX.sym('v')          # Velocity
        s = ca.SX.sym('s')          # Longitudinal position along path
        states = ca.vertcat(d, psi, v, s)

        delta = ca.SX.sym('delta')  # steering angle
        a = ca.SX.sym('a')          # acceleration
        controls = ca.vertcat(delta, a)

        # kappa = ca.SX.sym('kappa')  # Path curvature at position s
        # Define kappa(s) using CasADi's polyval function
        kappa = ca.polyval(self.kappa_coeffs, s)

        # Define the right-hand side of the model
        rhs = ca.vertcat(
            v * ca.sin(psi),
            (v / self.L) * ca.tan(delta) - (v * kappa * ca.cos(psi)) / (1 - d * kappa),
            a,
            (v * ca.cos(psi)) / (1 - d * kappa)
        )

        # Create a function for the model
        self.f = ca.Function('f', [states, controls], [rhs])

        # Initialize decision variables
        U = ca.SX.sym('U', self.NU, self.N)   # Control trajectory
        X = ca.SX.sym('X', self.NX, self.N+1) # State trajectory

        # Parameters (initial state and reference trajectory)
        P = ca.SX.sym('P', self.NX + self.N * self.NX)# Parameters (initial state + reference trajectory)

        # Objective function
        obj = 0                         
        

        # Constraints vecor
        g = []

        # Add initial state constraint
        g.append(X[:,0] - P[:self.NX])

        for k in range(self.N):
            # Statess at time k
            st = X[:,k]             # States at time k
            con = U[:,k]            # Control at time k

            # Reference states at time k
            x_ref = P[self.NX + + self.NX * k : self.NX + self.NX * (k+1)]
            # x_ref = ca.vertcat(0, 0, v_ref, 0)

            if k > 0:
                con_prev = U[:, k-1]
            else:
                con_prev = U[:, k]

            # Cost function
            obj += ca.mtimes([(con - con_prev).T, self.Rd, (con - con_prev)]) + ca.mtimes([(st - x_ref).T, self.Q, (st - x_ref)]) + ca.mtimes([con.T, self.R, con])

            # Model equations
            st_next = X[:,k+1]
            f_value = self.f(st, con)
            st_next_euler = st + self.dt * f_value

            # Add equality constraint
            g.append(st_next - st_next_euler)

        # Flatten constraints
        g = ca.vertcat(*g)

        # Decision Variables and parameters
        OPT_variables = ca.vertcat(
            ca.reshape(U, -1, 1),
            ca.reshape(X, -1, 1)
        )

        # Define the NLP (nonlinear programming) Problem
        nlp_prob = {
            'f': obj,
            'x': OPT_variables,
            'g': g,
            'p': P
        }

        # Set IPOPT solver options
        opts = {
            'ipopt.max_iter': 500,
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.acceptable_tol': 1e-8,
            'ipopt.acceptable_obj_change_tol': 1e-6
        }

        # Create solver instance
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

        # Total number of decision variables
        self.n_vars = OPT_variables.size()[0]

        # Upper and lower bounds on controls
        self.lbx = []
        self.ubx = []

        for _ in range(self.N):
            self.lbx.extend([self.delta_min, self.a_min])
            self.ubx.extend([self.delta_max, self.a_max])

        # Add bounds for states (Not implemented yet, fix as needed)
        for _ in range(self.N + 1):
            self.lbx.extend([-np.inf, -np.inf, self.v_min, -np.inf])
            self.ubx.extend([np.inf, np.inf, self.v_max, np.inf])

        # Convert lists to numpy arrays
        self.lbx = np.array(self.lbx).flatten()
        self.ubx = np.array(self.ubx).flatten()

        # Define constraint bounds
        self.lbg = np.zeros(g.size()[0])
        self.ubg = np.zeros(g.size()[0])

        assert len(self.lbx) == self.n_vars
        assert len(self.ubx) == self.n_vars

        # assert len(self.lbg) == g.size()[0]
        # assert len(self.ubg) == g.size()[0]

    # Implement MPC loop
    def solve_mpc(self, x0, x_ref_traj, u0):
        # Adjust initial control guess
        if u0.shape[0] != self.N * self.NU:
            u0 = np.zeros(self.N * self.NU)

        # Concatenate parameters
        p = np.concatenate((x0, x_ref_traj.flatten()), axis=0)


        # Initial guess for decision variables
        init_control = u0.reshape(-1, 1)
        init_state = x0.reshape(-1, 1)


        # Initial guess for optimization variables
        x_init = np.concatenate((init_control, np.tile(init_state, (self.N + 1, 1))), axis=0)

        # Solve the optimization problem
        sol = self.solver(
            x0=x_init,
            lbx=self.lbx,
            ubx=self.ubx,
            lbg=self.lbg,
            ubg=self.ubg,
            p=p
        )

        # Extract optimal control input\
        u_opt = sol['x'][:self.N * self.NU]
        u_opt = np.array(u_opt.full()).flatten()

        # Extract the predicted state trajectory
        x_pred = sol['x'][self.N * self.NU:]
        x_pred = np.array(x_pred.full()).reshape(self.N + 1, self.NX)

        return u_opt, x_pred