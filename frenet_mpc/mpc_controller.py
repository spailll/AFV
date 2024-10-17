import casadi as ca
import numpy as np
# import matplotlib.pyplot as plt

class MPCController:
    def __init__(self, path_planner, vehicle_model, N=20, dt=0.1, debug=False):
    # def __init__(self, N, dt, kappa_coeffs, delta_max_degree=30):
        # MPC parameters
        self.N = N
        self.dt = dt
        self.path_planner = path_planner
        self.vehicle = vehicle_model
        self.debug = debug

        self.NX = 4
        self.NU = 2

        self.v_max = 15.0
        self.v_min = 0.0

        self.setup_mpc()



    def setup_mpc(self):
        self.d = ca.SX.sym('d')
        self.psi = ca.SX.sym('psi')
        self.v = ca.SX.sym('v')
        self.s = ca.SX.sym('s')
        self.states = ca.vertcat(self.d, self.psi, self.v, self.s)

        self.delta = ca.SX.sym('delta')
        self.a = ca.SX.sym('a')
        self.controls = ca.vertcat(self.delta, self.a)

        L = self.vehicle.L
        print(self.path_planner.kappa_coeffs)
        self.kappa_coeffs = ca.DM(self.path_planner.kappa_coeffs)
        kappa = ca.polyval(self.path_planner.kappa_coeffs, self.s)
        epsilon = 1e-4  # Small value to avoid division by zero

        d_dot = self.v * ca.sin(self.psi)
        psi_dot = (self.v / L) * ca.tan(self.delta) - (self.v * kappa * ca.cos(self.psi)) / (1 - self.d * kappa + epsilon)
        v_dot = self.a
        s_dot = (self.v * ca.cos(self.psi)) / (1 - self.d * kappa + epsilon)

        self.f = ca.Function('f', [self.states, self.controls], [ca.vertcat(d_dot, psi_dot, v_dot, s_dot)])

        U = ca.SX.sym('U', self.NU, self.N)
        X = ca.SX.sym('X', self.NX, self.N+1)

        P = ca.SX.sym('P', 4 + 4*self.N)

        obj = 0
        g = []

        Q = ca.diag([20.0, 20.0, 0.1, 0.0])
        R = ca.diag([0.1, 0.1])
        Rd = ca.diag([5.0, 5.0])

        g.append(X[:,0] - P[0:4])

        for k in range(self.N):
            d_ref = P[4 + 4*k]
            psi_ref = P[4 + 4*k + 1]
            v_ref = P[4 + 4*k + 2]
            s_ref = P[4 + 4*k + 3]
            ref_state = ca.vertcat(d_ref, psi_ref, v_ref, s_ref)

            state_error = X[:,k] - ref_state
            obj += ca.mtimes([state_error.T, Q, state_error])

            control = U[:,k]
            obj += ca.mtimes([control.T, R, control])

            control_prev = U[:,k-1] if k > 0 else U[:,k]
            control_change = control - control_prev
            obj += ca.mtimes([control_change.T, Rd, control_change])

            st_next = X[:,k+1]
            f_value = self.f(X[:,k], U[:,k])
            st_next_euler = X[:,k] + self.dt * f_value
            g.append(st_next - st_next_euler)

        g = ca.vertcat(*g)

        OPT_variables = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(X, -1, 1))

        nlp_prob = {
            'f': obj,
            'x': OPT_variables,
            'g': g,
            'p': P
        }

        opts = {
            'ipopt.max_iter': 1000,
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.acceptable_tol': 1e-6,
            'ipopt.acceptable_obj_change_tol': 1e-4
        }

        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

        self.lbx = []
        self.ubx = []

        delta_max = np.deg2rad(30)
        delta_min = -delta_max
        a_max = 5.0
        a_min = -5.0

        for _ in range(self.N):
            self.lbx.extend([delta_min, a_min])
            self.ubx.extend([delta_max, a_max])

        for _ in range(self.N + 1):
            self.lbx.extend([-np.inf, -np.inf, self.v_min, 0.0])
            self.ubx.extend([np.inf, np.inf, self.v_max, np.inf])

        self.lbx = np.array(self.lbx).flatten()
        self.ubx = np.array(self.ubx).flatten()

        self.lbg = np.zeros(g.size()[0])
        self.ubg = np.zeros(g.size()[0])

    def generate_reference_trajectory(self, frenet_state):
        d, psi, v, s = frenet_state
        s_future = s + np.cumsum([self.dt * v] * self.N)

        d_ref = np.zeros(self.N)
        psi_ref = np.zeros(self.N)
        v_ref = np.full(self.N, v)
        s_ref = s_future

        ref_traj = np.vstack([d_ref, psi_ref, v_ref, s_ref])
        # if self.debug:
            # print("Reference Trajectory Generated: ", ref_traj)
        return ref_traj

    def solve(self, frenet_state, reference_traj):
        p = np.concatenate((frenet_state, reference_traj.flatten()))

        U0 = np.zeros((2, self.N))
        X0 = np.tile(frenet_state.reshape(-1, 1), (1, self.N+1))
        opt_init = np.concatenate((U0.flatten(), X0.flatten()))

        sol = self.solver(
            x0=opt_init,
            lbx=self.lbx,
            ubx=self.ubx,
            lbg=self.lbg,
            ubg=self.ubg,
            p=p
        )

        sol_u = sol['x'][:2*self.N].full().flatten()
        sol_x = sol['x'][2*self.N:].full().flatten()

        delta_opt = sol_u[0]
        a_opt = sol_u[1]

        if self.debug:
            print(f"Optimal Control Inputs -> Delta: {delta_opt:.3f} rad, Accel: {a_opt:.3f}")
            # print(f"Objective Function Value: {sol['f'].full()[0][0]:.3f}")

        # print(f"Optimal Control Inputs -> Delta: {delta_opt:.3f} rad, Accel: {a_opt:.3f} m/s²")
        # # Optionally, print the cost
        # print(f"Objective Function Value: {sol['f'].full()[0][0]:.3f}")

        return delta_opt, a_opt



