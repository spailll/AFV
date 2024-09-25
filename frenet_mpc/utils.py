import numpy as np

def global_to_frenet(x, y, theta, path_planner):
    dx = path_planner.x_ref - x
    dy = path_planner.y_ref - y
    dist_squared = dx**2 + dy**2
    min_idx = np.argmin(dist_squared)

    path_theta = path_planner.theta_ref[min_idx]
    d = (x - path_planner.x_ref[min_idx]) * (-np.sin(path_theta)) + (y - path_planner.y_ref[min_idx]) * np.cos(path_theta)

    psi = theta - path_theta
    psi = (psi + np.pi) % (2 * np.pi) - np.pi # Normalize to [-pi, pi]

    s = path_planner.s_ref[min_idx]
    
    return d, psi, s

def frenet_to_global(frenet_state, path_planner):
    d, psi, v, s = frenet_state
    x_path = np.interp(s, path_planner.s_ref, path_planner.x_ref)
    y_path = np.interp(s, path_planner.s_ref, path_planner.y_ref)
    theta_path = np.interp(s, path_planner.s_ref, path_planner.theta_ref)

    x = x_path - d * np.sin(theta_path)
    y = y_path + d * np.cos(theta_path)
    theta = theta_path + psi
    theta = (theta + np.pi) % (2 * np.pi) - np.pi # Normalize to [-pi, pi]

    return x, y, theta, v

def find_closest_path_point(vehicle_state, path_planner):
    closest_dist = float('inf')
    closest_s = 0.0
    closest_theta = 0.0
    d = 0.0
    psi = 0.0

    x, y, theta, v = vehicle_state

    for i in range(len(path_planner.x_ref) - 1):
        x1, y1 = path_planner.x_ref[i], path_planner.y_ref[i]
        x2, y2 = path_planner.x_ref[i+1], path_planner.y_ref[i+1]
        dx = x2 - x1
        dy = y2 - y1
        segment_length = np.hypot(dx, dy)
        
        if segment_length == 0:
            continue

        t = ((x - x1) * dx + (y - y1) * dy) / segment_length**2
        t = max(0.0, min(1.0, t))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        dist = np.hypot(x - proj_x, y - proj_y)

        if dist < closest_dist: 
            closest_dist = dist
            closest_s = path_planner.s_ref[i] + t * segment_length
            path_theta = path_planner.theta_ref[i]
            d = (x - proj_x) * (-np.sin(path_theta)) + (y - proj_y) * np.cos(path_theta)
            psi = theta - closest_theta
            psi = (psi + np.pi) % (2 * np.pi) - np.pi # Normalize to [-pi, pi]
    closest_s = np.clip(closest_s, 0, path_planner.s_ref[-1])
    
    return d, psi, closest_s


def compute_desired_speed(x_ref, y_ref, v_max, v_min):
    dx = np.gradient(x_ref)
    dy = np.gradient(y_ref)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)

    curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5
    curvature = np.nan_to_num(curvature)

    curvature_max = np.max(curvature) if np.max(curvature) != 0 else 1

    v_des = v_max * (1 - curvature / curvature_max)
    v_des = np.clip(v_des, v_min, v_max)

    return v_des


