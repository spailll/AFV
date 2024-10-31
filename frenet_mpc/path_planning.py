import numpy as np

class PathPlanner:
    def __init__(self):
        self.x_ref = []
        self.y_ref = []
        self.s_ref = []
        self.v_ref = []
        self.theta_ref = []
        self.kappa_ref = []
        self.kappa_coeffs = []

    def set_path(self, x_ref, y_ref):
        self.x_ref = x_ref
        self.y_ref = y_ref
        self.s_ref = self.compute_cumulative_arc_length(self.x_ref, self.y_ref)
        self.theta_ref = self.compute_path_heading(self.x_ref, self.y_ref)
        self.kappa_ref = self.compute_path_curvature(self.x_ref, self.y_ref, self.s_ref)
        self.kappa_coeffs = self.fit_curvature_polynomial(self.s_ref, self.kappa_ref)

    def compute_cumulative_arc_length(self, x_ref, y_ref):
        dx = np.diff(x_ref)
        dy = np.diff(y_ref)
        ds = np.hypot(dx, dy)
        s_ref = np.concatenate(([0], np.cumsum(ds)))
        return s_ref

    def compute_path_heading(self, x_ref, y_ref):
        partial_x = np.gradient(x_ref)
        partial_y = np.gradient(y_ref)
        theta_ref = np.arctan2(partial_y, partial_x)
        return theta_ref

    def compute_path_curvature(self, x_ref, y_ref, s_ref):
        dx_ds = np.gradient(x_ref, s_ref)
        dy_ds = np.gradient(y_ref, s_ref)

        d2x_ds2 = np.gradient(dx_ds, s_ref)
        d2y_ds2 = np.gradient(dy_ds, s_ref)

        kappa_ref = (dx_ds * d2y_ds2 - dy_ds * d2x_ds2) / (dx_ds**2 + dy_ds**2)**1.5
        kappa_ref = np.nan_to_num(kappa_ref)
        return kappa_ref

    def fit_curvature_polynomial(self, s_ref, kappa_ref, degree=5):
        coeffs = np.polyfit(s_ref, kappa_ref, deg=degree)
        return coeffs

    def set_speed_profile(self, v_ref):
        self.v_ref = v_ref


