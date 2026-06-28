import math
import time
import numpy as np

from tactical_brain.A_planner import XY_RESOLUTION, ENEMY_TTL

# קבועים
R_ENEMY = 0.5  # meters
R_TEAMMATE = 0.3  # meters


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


def line_of_sight_clear(start, end, walls_set):
    dist = distance(start, end)
    num_points = int(dist * 10) + 1
    x_points = np.linspace(start[0], end[0], num_points)
    y_points = np.linspace(start[1], end[1], num_points)

    for x, y in zip(x_points, y_points):
        grid_x, grid_y = round(x), round(y)
        if (grid_x, grid_y) in walls_set:
            return False
    return True


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])
