import numpy as np

def generate_path_from_waypoints(waypoints, corner_radius=5.0, num_points_per_arc=20):
    """
    Generates a continuous path from discrete waypoints by keeping straight segments
    and rounding corners with circular arcs.

    :param waypoints: An array of waypoints, each waypoint is a [x, y] pair.
    :param corner_radius: Radius of the circular arcs used to round corners.
    :param num_points_per_arc: Number of points to generate along each arc.
    :return: Tuple of arrays (path_x, path_y) representing the continuous path.
    """
    path_x = []
    path_y = []
    num_waypoints = len(waypoints)

    if num_waypoints < 2:
        raise ValueError("At least two waypoints are required to generate a path.")

    # Initialize the starting point
    path_x.append(waypoints[0][0])
    path_y.append(waypoints[0][1])

    for i in range(1, num_waypoints - 1):
        p_prev = np.array(waypoints[i - 1])
        p_curr = np.array(waypoints[i])
        p_next = np.array(waypoints[i + 1])

        # Calculate direction vectors
        v_in = p_curr - p_prev
        v_out = p_next - p_curr

        # Normalize direction vectors
        v_in_norm = v_in / np.linalg.norm(v_in)
        v_out_norm = v_out / np.linalg.norm(v_out)

        # Calculate angle between incoming and outgoing segments
        cos_theta = np.dot(v_in_norm, v_out_norm)
        theta = np.arccos(np.clip(cos_theta, -1.0, 1.0))

        # Calculate distance from corner to tangent points
        tan_half_theta = np.tan(theta / 2)
        if tan_half_theta == 0:
            d = 0
        else:
            d = corner_radius / tan_half_theta

        # Limit d to avoid overlap
        d = min(d, np.linalg.norm(v_in) / 2, np.linalg.norm(v_out) / 2)

        # Calculate tangent points
        tangent_in = p_curr - v_in_norm * d
        tangent_out = p_curr + v_out_norm * d

        # Append straight path from last point to tangent_in
        last_point = np.array([path_x[-1], path_y[-1]])
        straight_segment_in = np.linspace(last_point, tangent_in, int(np.linalg.norm(tangent_in - last_point)))
        path_x.extend(straight_segment_in[:, 0])
        path_y.extend(straight_segment_in[:, 1])

        # Calculate arc center
        # Normal vectors
        n_in = np.array([-v_in_norm[1], v_in_norm[0]])
        n_out = np.array([-v_out_norm[1], v_out_norm[0]])

        # Bisector
        bisector = (v_in_norm + v_out_norm) / np.linalg.norm(v_in_norm + v_out_norm)
        bisector_perp = np.array([-bisector[1], bisector[0]])

        # Determine the sign for the bisector
        sign = np.sign(np.cross(v_in_norm, v_out_norm))
        if sign == 0:
            sign = 1.0  # Default to 1 if vectors are colinear

        # Center of the arc
        center = p_curr + sign * bisector_perp * (corner_radius / np.sin(theta / 2))

        # Angles for the arc
        start_angle = np.arctan2(tangent_in[1] - center[1], tangent_in[0] - center[0])
        end_angle = np.arctan2(tangent_out[1] - center[1], tangent_out[0] - center[0])

        # Adjust angles to ensure correct arc direction
        if sign > 0:
            if end_angle < start_angle:
                end_angle += 2 * np.pi
        else:
            if end_angle > start_angle:
                end_angle -= 2 * np.pi

        arc_angles = np.linspace(start_angle, end_angle, num_points_per_arc)
        arc_x = center[0] + corner_radius * np.cos(arc_angles)
        arc_y = center[1] + corner_radius * np.sin(arc_angles)
        path_x.extend(arc_x)
        path_y.extend(arc_y)

    # Handle the last segment
    p_last_tangent = np.array([path_x[-1], path_y[-1]])
    p_last_waypoint = np.array(waypoints[-1])
    straight_segment_out = np.linspace(p_last_tangent, p_last_waypoint, int(np.linalg.norm(p_last_waypoint - p_last_tangent)))
    path_x.extend(straight_segment_out[:, 0])
    path_y.extend(straight_segment_out[:, 1])

    return np.array(path_x), np.array(path_y)
