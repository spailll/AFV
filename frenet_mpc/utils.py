import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

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

def densify_path(waypoints, spacing=10.0, tolerance=1e-6):
    densified_waypoints = [waypoints[0]]

    for i in range(1, len(waypoints)):
        start = waypoints[i - 1]
        end = waypoints[i]
        segment = end - start
        distance = np.linalg.norm(segment)

        if distance < tolerance:
            continue

        num_points = int(np.floor(distance / spacing))
        
        if num_points > 0:
            unit_vector = segment / distance

            for j in range(1, num_points + 1):
                point = start + unit_vector * j * spacing
                densified_waypoints.append(point)
        densified_waypoints.append(end)

    densified_waypoints = np.array(densified_waypoints)
    # densified_waypoints = remove_duplicate_waypoints(densified_waypoints, tolerance=tolerance)
    return densified_waypoints

# def remove_duplicate_waypoints(waypoints, tolerance=1e-3):
#     if len(waypoints) == 0:
#         return waypoints
#     unique_waypoints = [waypoints[0]]
#     for point in waypoints[1:]:
#         if np.linalg.norm(point - unique_waypoints[-1]) > tolerance:
#             unique_waypoints.append(point)
#     return np.array(unique_waypoints)

def smooth_path_with_splines(waypoints):
    distances = np.sqrt(np.sum(np.diff(waypoints, axis=0)**2, axis=1))
    s = np.insert(np.cumsum(distances), 0, 0)

    # Extract x and y coordinates
    x = waypoints[:, 0]
    y = waypoints[:, 1]

    # Create cubic splines for x(s) and y(s)
    cs_x = CubicSpline(s, x)
    cs_y = CubicSpline(s, y)

    # Define fine sampling along the path
    s_fine = np.linspace(s[0], s[-1], 1000)

    # Generate interpolated coordinates
    x_fine = cs_x(s_fine)
    y_fine = cs_y(s_fine)

    return s_fine, x_fine, y_fine, cs_x, cs_y

def detect_turns(densified_waypoints, angle_threshold_deg=45):
    turn_indices = []
    angle_threshold_rad = np.deg2rad(angle_threshold_deg)

    for i in range(1, len(densified_waypoints)-1):
        prev = densified_waypoints[i-1]
        current = densified_waypoints[i]
        next_pt = densified_waypoints[i+1]

        v1 = current - prev
        v2 = next_pt - current

        # Normalize vectors
        v1_norm = v1 / np.linalg.norm(v1) if np.linalg.norm(v1) != 0 else v1
        v2_norm = v2 / np.linalg.norm(v2) if np.linalg.norm(v2) != 0 else v2

        # Compute angle between vectors
        dot_prod = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
        angle = np.arccos(dot_prod)

        if angle > angle_threshold_rad:
            turn_indices.append(i)

    return turn_indices

def apply_curvature(densified_waypoints, turn_indices, radius=10, num_curve_points=20):
    rounded_waypoints = []
    skip_indices = set()

    # N = len(densified_waypoints)
    
    for i in range(len(densified_waypoints)):
        if i in skip_indices: 
            continue

        if i in turn_indices:
            # Determine turn direction
            prev = densified_waypoints[i-1]
            current = densified_waypoints[i]
            next_pt = densified_waypoints[i+1]

            v1 = current - prev
            v2 = next_pt - current

            norm_v1 = np.linalg.norm(v1)
            norm_v2 = np.linalg.norm(v2)
            if norm_v1 == 0 or norm_v2 == 0:
                continue # Avoid division by zero
            # Normalize vectors
            v1_norm = v1 / norm_v1 
            v2_norm = v2 / norm_v2

            # Compute cross product to determine turn direction
            cross_prod = v1_norm[0] * v2_norm[1] - v1_norm[1] * v2_norm[0]

            if cross_prod > 0:
                turn_direction = 'left'
                perp = np.array([-v1_norm[1], v1_norm[0]])  # Left perpendicular
            elif cross_prod < 0:
                turn_direction = 'right'
                perp = np.array([v1_norm[1], -v1_norm[0]])  # Right perpendicular
            else:
                turn_direction = 'straight'
                perp = np.array([0, 0])

            if turn_direction == 'straight':
                rounded_waypoints.append(current)
                continue

            # Calculate tangent length
            angle = np.arccos(np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0))
            tangent_length = radius / np.tan(angle / 2)

            # Ensure that the tangent length does not exceed the segment length
            segment_length = min(norm_v1, norm_v2)
            if tangent_length > segment_length / 2:
                tangent_length = segment_length / 2  # Adjust to half the segment length

            # Start and end points of the curve
            start_curve = current - v1_norm * tangent_length
            end_curve = current + v2_norm * tangent_length

            # Compute center of the curve
            center = current + perp * radius

            # Compute angles for the arc
            theta_start = np.arctan2(start_curve[1] - center[1], start_curve[0] - center[0])
            theta_end = np.arctan2(end_curve[1] - center[1], end_curve[0] - center[0])

            # Adjust theta_end based on turn direction to ensure correct arc direction
            if turn_direction == 'left' and theta_end < theta_start:
                theta_end += 2 * np.pi
            elif turn_direction == 'right' and theta_end > theta_start:
                theta_end -= 2 * np.pi

            # Generate points along the arc
            theta = np.linspace(theta_start, theta_end, num_curve_points)
            curve_points = center + radius * np.stack((np.cos(theta), np.sin(theta)), axis=1)

            # Append start_curve, curve_points, end_curve
            rounded_waypoints.append(start_curve)
            rounded_waypoints.extend(curve_points)
            rounded_waypoints.append(end_curve)
        else:
            # Avoid duplicating points if already appended in the curve
            if i > 0 and (i-1) in turn_indices:
                continue
            rounded_waypoints.append(densified_waypoints[i])

    return np.array(rounded_waypoints)

def interpolate_spline(rounded_waypoints, num_fine_points=1000):
    # Calculate cumulative distance (s) along the waypoints
    distances = np.sqrt(np.sum(np.diff(rounded_waypoints, axis=0)**2, axis=1))
    s = np.insert(np.cumsum(distances), 0, 0)  # Insert 0 at the beginning
    
    # diff_s = np.diff(s)
    # non_increasing = np.where(diff_s <= 0)[0]
    # print("Non-increasing at indices:", non_increasing)


    # Extract x and y coordinates
    x = rounded_waypoints[:, 0]
    y = rounded_waypoints[:, 1]

    # Create cubic splines for x(s) and y(s)
    cs_x = CubicSpline(s, x)
    cs_y = CubicSpline(s, y)

    # Define fine sampling along the path
    s_fine = np.linspace(s[0], s[-1], num_fine_points)

    # Generate interpolated coordinates
    x_fine = cs_x(s_fine)
    y_fine = cs_y(s_fine)

    return s_fine, x_fine, y_fine, cs_x, cs_y


def plot_path(original_waypoints, densified_waypoints, rounded_waypoints, x_fine, y_fine):
    plt.figure(figsize=(14, 10))
    
    # Original Waypoints
    plt.plot(original_waypoints[:, 0], original_waypoints[:, 1], 'ro-', label='Original Waypoints')
    
    # Densified Path
    plt.plot(densified_waypoints[:, 0], densified_waypoints[:, 1], 'g.', markersize=2, label='Densified Path')
    
    
    # Rounded Waypoints
    plt.plot(rounded_waypoints[:, 0], rounded_waypoints[:, 1], 'bx--', label='Rounded Corners')
    
    # Final Interpolated Path
    plt.plot(x_fine, y_fine, 'k-', label='Final Smooth Path')
    
    plt.title('Path Smoothing with Densification and Rounded Corners')
    plt.xlabel('X [meters]')
    plt.ylabel('Y [meters]')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')  # Ensures equal scaling for both axes
    plt.show()

def is_target_reached(current_pos, target_pos, distance_threshold=1.0):
    distance = np.linalg.norm(target_pos - current_pos)
    return distance <= distance_threshold

