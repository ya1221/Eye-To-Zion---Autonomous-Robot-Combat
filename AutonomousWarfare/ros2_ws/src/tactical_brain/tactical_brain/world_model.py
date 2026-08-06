import math
import time
import numpy as np

from tactical_brain.A_planner import XY_RESOLUTION, ENEMY_TTL

# קבועים
R_ENEMY = 0.5  # meters
R_TEAMMATE = 0.3  # meters
# Separate from A_planner.ENEMY_TTL (which ages danger-grid cells out over
# 15s) - this is how long a single enemy detection (no "no enemy visible"
# message exists upstream) stays believed before being dropped outright.
ENEMY_MEMORY_TIMEOUT = 2.0

# Visibility (enemy line-of-sight) endpoint margin [meters]. A detected
# enemy's position IS a lidar return the SLAM map marks occupied (that's
# literally how it was detected), and the robot's own footprint often is
# too - so a ray sampled right up to each endpoint reports itself
# "blocked" by its own start/end, making the LOS check return False
# essentially always. The visibility caller passes this (converted to grid
# cells) so samples within it of either endpoint are ignored - only walls
# strictly BETWEEN the two ends occlude sight. Kept off (0.0) by default so
# get_danger_dict's own line_of_sight_clear use is unchanged.
LOS_ENDPOINT_MARGIN_METERS = 0.3


def prune_stale_enemies(enemies_by_detector, memory_timeout=ENEMY_MEMORY_TIMEOUT):
    # Keyed by whichever robot_id reported each sighting (my own onboard
    # detection, or a teammate's broadcast) - a dict rather than a flat list
    # so two robots simultaneously tracking different enemies don't clobber
    # each other.
    current_time = time.time()
    return {
        detector_id: enemy for detector_id, enemy in enemies_by_detector.items()
        if current_time - enemy.get("timestamp", current_time) < memory_timeout
    }


def get_danger_dict(danger_dict, enemies_list, static_obstacles):
    for enemy_data in enemies_list:
        enemy_x = int(enemy_data["x"] / XY_RESOLUTION)
        enemy_y = int(enemy_data["y"] / XY_RESOLUTION)
        r_enemy_squares = int(R_ENEMY / XY_RESOLUTION)

        for x in range(enemy_x - r_enemy_squares, enemy_x + r_enemy_squares + 1):
            for y in range(enemy_y - r_enemy_squares, enemy_y + r_enemy_squares + 1):
                if (distance((enemy_x, enemy_y), (x, y)) <= r_enemy_squares
                    and line_of_sight_clear((enemy_x, enemy_y), (x, y), static_obstacles)):
                    danger_dict[(x, y)] = enemy_data["timestamp"]

    return danger_dict


def get_current_time():
    return time.time()


def delete_old_danger_squares(danger_dict):
    current_time = get_current_time()
    for square in list(danger_dict.keys()):
        if current_time - danger_dict[square] >= ENEMY_TTL:
            danger_dict.pop(square)

    return danger_dict


def get_teammates_aura_set(teammates_dict):
    teammates_aura_set = set()
    for teammate_data in teammates_dict.values():
        teammate_x = int(teammate_data["x"] / XY_RESOLUTION)
        teammate_y = int(teammate_data["y"] / XY_RESOLUTION)
        r_teammate_squares = int(R_TEAMMATE / XY_RESOLUTION)

        for x in range(teammate_x - r_teammate_squares, teammate_x + r_teammate_squares + 1):
            for y in range(teammate_y - r_teammate_squares, teammate_y + r_teammate_squares + 1):
                    if distance((teammate_x, teammate_y), (x, y)) <= r_teammate_squares:
                        teammates_aura_set.add((x, y))

    return teammates_aura_set


def line_of_sight_clear(start, end, walls_set, endpoint_margin_cells=0.0):
    dist = distance(start, end)
    if dist == 0:
        return True
    num_points = int(dist * 10) + 1
    x_points = np.linspace(start[0], end[0], num_points)
    y_points = np.linspace(start[1], end[1], num_points)

    for x, y in zip(x_points, y_points):
        # Ignore samples within endpoint_margin_cells of either endpoint -
        # the endpoints are the robot's own footprint and the enemy's body
        # (a lidar return SLAM marks occupied), which would otherwise make
        # every ray block on itself. See LOS_ENDPOINT_MARGIN_METERS.
        if endpoint_margin_cells > 0.0 and (
            distance((x, y), start) <= endpoint_margin_cells
            or distance((x, y), end) <= endpoint_margin_cells
        ):
            continue
        grid_x, grid_y = round(x), round(y)
        if (grid_x, grid_y) in walls_set:
            return False
    return True


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])
