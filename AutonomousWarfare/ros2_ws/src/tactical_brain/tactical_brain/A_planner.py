import math
import heapq
import numpy as np
import time

# === הגדרות מערכת ורובוט ===
XY_RESOLUTION = 0.1   # רזולוציית רשת [מטרים]
YAW_RESOLUTION = math.radians(5.0) # רזולוציית זווית [רדיאנים]
MOVE_STEP = 0.2       # מרחק תנועה בכל צעד [מטרים]
TURNING_RADIUS = 0.7  # רדיוס סיבוב מינימלי של הרובוט [מטרים]
ROBOT_RADIUS = 0.25   # רדיוס פיזי של הרובוט להתנגשויות [מטרים]
# Nav2's keepout requires a larger planning clearance to prevent A* from proposing unreachable goals.
# Empirically set to 0.40m to balance safety and search space.
PLANNING_CLEARANCE = 0.40
BUCKET_CELLS = math.ceil(PLANNING_CLEARANCE / XY_RESOLUTION) + 1  # spatial-index bucket size for check_collision

# Plannable-region geofence limits to keep the robot from planning off the drivable arena. Configurable via set_arena_bounds() but defaults to the 5m sim maze dimensions.
ARENA_MIN = 0.1
ARENA_MAX = 4.9


def set_arena_bounds(size_meters, margin=0.1):
    """Sets the plannable-region geofence size for the real arena. Call once at startup to override the sim maze defaults."""
    global ARENA_MIN, ARENA_MAX
    ARENA_MIN = margin
    ARENA_MAX = size_meters - margin

# === פרמטרי עלויות (Costs) ===
H_WEIGHT = 1.2        # משקל ההיוריסטיקה (Weighted A*) - מאוזן למניעת תקיעה
# Minimum consecutive steps a direction run must last before a free gear switch to prevent degenerate path segments.
MIN_RUN_STEPS = 3
SHORT_RUN_PENALTY = MOVE_STEP * 3.0
TEAMMATE_DISCOUNT = MOVE_STEP * 0.5
ENEMY_MAX_COST = 1.0  # קנס אויב נמוך לעידוד אגרסיביות (חציית שטח השמדה)
ENEMY_TTL = 15        # זמן חיים של זיהוי אויב [שניות]

class Node:
    def __init__(self, x_ind, y_ind, yaw_ind, direction, p_node, cost, steering=0, run_length=1):
        self.x_ind = x_ind
        self.y_ind = y_ind
        self.yaw_ind = yaw_ind
        self.direction = direction # 1 קדימה, -1 אחורה
        self.p_node = p_node
        self.cost = cost
        self.steering = steering
        # Tracks consecutive steps in the current direction to penalize excessive gear switching and avoid degenerate segments.
        self.run_length = run_length

    def __lt__(self, other):
        return self.cost < other.cost

# =========================================================
#  מפת היוריסטיקה (Dijkstra)
# =========================================================
def calc_holonomic_heuristic_with_obstacle(goal_node, obstacle_set, xy_resolution):
    min_x, min_y = 0, 0
    max_x, max_y = int(6.0 / xy_resolution), int(6.0 / xy_resolution)
    heuristic_map = {} 
    
    gx, gy = goal_node.x_ind, goal_node.y_ind
    pq = [(0, gx, gy)]
    heuristic_map[(gx, gy)] = 0
    
    motion = [
        (1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0),
        (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
    ]
    
    while pq:
        cost, cx, cy = heapq.heappop(pq)
        if cost > heuristic_map.get((cx, cy), float('inf')):
            continue

        for dx, dy, move_cost in motion:
            nx, ny = cx + dx, cy + dy
            if nx < min_x or nx > max_x or ny < min_y or ny > max_y:
                continue
            if (nx, ny) in obstacle_set:
                continue
            new_cost = cost + move_cost
            if new_cost < heuristic_map.get((nx, ny), float('inf')):
                heuristic_map[(nx, ny)] = new_cost
                heapq.heappush(pq, (new_cost, nx, ny))
                
    return heuristic_map

# =========================================================
#  האלגוריתם המרכזי - Hybrid A*
# =========================================================
def calc_hybrid_a_star(start, goal, obstacle_set, xy_resolution, yaw_resolution, danger_dict, teammates_aura_set):
    sx_ind = int(round(start[0] / xy_resolution))
    sy_ind = int(round(start[1] / xy_resolution))
    syaw_ind = int(round(start[2] / yaw_resolution))
    
    gx_ind = int(round(goal[0] / xy_resolution))
    gy_ind = int(round(goal[1] / xy_resolution))
    gyaw_ind = int(round(goal[2] / yaw_resolution))
    
    start_node = Node(sx_ind, sy_ind, syaw_ind, 1, None, 0)
    goal_node = Node(gx_ind, gy_ind, gyaw_ind, 1, None, 0)
    
    h_map = calc_holonomic_heuristic_with_obstacle(goal_node, obstacle_set, xy_resolution)
    spatial_index = build_spatial_index(obstacle_set)

    open_set = { (sx_ind, sy_ind, syaw_ind): start_node }
    closed_set = {}
    pq = [] 
    heapq.heappush(pq, (0, start_node))
    
    iter_count = 0
    max_iter = 100000
    current_time = time.time()

    while True:
        if not pq: return None
        if iter_count > max_iter: return None
        iter_count += 1
        
        _, current = heapq.heappop(pq)
        
        dist_to_goal = math.hypot(current.x_ind - goal_node.x_ind, current.y_ind - goal_node.y_ind) * xy_resolution
        # Requires the final approach step to be forward to avoid Gazebo ODE friction stalls when reversing near the goal.
        if dist_to_goal <= 0.1 and current.direction == 1:
            rx, ry, rt, rd = extract_path(current, xy_resolution, yaw_resolution)
            # Stops expanding within 0.1m and snaps the last waypoint to the goal to ensure the path is geometrically complete.
            rx.append(goal[0])
            ry.append(goal[1])
            rt.append(rt[-1])
            rd.append(rd[-1])
            return ((rx, ry, rt, rd), current.cost)
        
        curr_id = (current.x_ind, current.y_ind, current.yaw_ind)
        if curr_id in closed_set: continue
        if curr_id in open_set: del open_set[curr_id]
        closed_set[curr_id] = current
        
        next_nodes = calc_motion(current, xy_resolution, yaw_resolution, danger_dict, teammates_aura_set, current_time)
        
        for next_node in next_nodes:
            if not check_collision(next_node, obstacle_set, xy_resolution, spatial_index):
                continue
            
            next_id = (next_node.x_ind, next_node.y_ind, next_node.yaw_ind)
            if next_id in closed_set: continue
            
            # חישוב היוריסטיקה (H)
            try:
                h_cost = h_map[(next_node.x_ind, next_node.y_ind)] * xy_resolution
            except KeyError:
                h_cost = math.hypot(next_node.x_ind - goal_node.x_ind, 
                                    next_node.y_ind - goal_node.y_ind) * xy_resolution
            
            # נוסחת ה-Weighted A*
            new_total_cost = next_node.cost + (h_cost * H_WEIGHT)
            
            if next_id not in open_set or open_set[next_id].cost > next_node.cost:
                open_set[next_id] = next_node
                heapq.heappush(pq, (new_total_cost, next_node))

# =========================================================
#  מודל תנועה (Ackermann Kinematics)
# =========================================================
def calc_motion(node, xy_res, yaw_res, danger_dict, teammates_aura_set, current_time):
    next_nodes = []
    steering_inputs = [-math.radians(20), 0, math.radians(20)]
    # Includes reverse direction to allow planning in tight spaces, while PathExecutor splits the path into per-direction segments.
    directions = [1, -1] # קדימה ואחורה

    current_yaw = node.yaw_ind * yaw_res
    current_x = node.x_ind * xy_res
    current_y = node.y_ind * xy_res
    
    for direction in directions:
        for steering in steering_inputs:
            # מודל אקרמן לסיבוב
            yaw = current_yaw + (direction * MOVE_STEP / TURNING_RADIUS) * math.tan(steering)
            x = current_x + direction * MOVE_STEP * math.cos(yaw)
            y = current_y + direction * MOVE_STEP * math.sin(yaw)
            
            x_ind = int(round(x / xy_res))
            y_ind = int(round(y / xy_res))
            yaw_ind = int(round(yaw / yaw_res))
            
            # חישוב עלויות צעד
            direction_cost = 0 if direction == 1 else MOVE_STEP * 1.0
            switching = direction != node.direction
            switch_gear_cost = 0 if not switching else MOVE_STEP * 2.0
            if switching and node.run_length < MIN_RUN_STEPS:
                # Adds an escalating penalty for switching gears before finishing a minimum-length leg to encourage longer runs.
                switch_gear_cost += (MIN_RUN_STEPS - node.run_length) * SHORT_RUN_PENALTY
            new_run_length = node.run_length + 1 if not switching else 1
            steer_cost = abs(steering) * MOVE_STEP * 0.5

            step_cost = MOVE_STEP + direction_cost + switch_gear_cost + steer_cost
            
            # הנחת חבר צוות
            if (x_ind, y_ind) in teammates_aura_set:
                step_cost -= TEAMMATE_DISCOUNT

            # קנס אויב עם דעיכה בזמן
            if (x_ind, y_ind) in danger_dict:
                time_since_danger = current_time - danger_dict[(x_ind, y_ind)]
                if time_since_danger < ENEMY_TTL:
                    # קנס שדועך מ-1.0 לאפס בתוך 15 שניות
                    enemy_penalty = (-ENEMY_MAX_COST / ENEMY_TTL) * time_since_danger + ENEMY_MAX_COST
                    step_cost += max(0, enemy_penalty)

            next_nodes.append(Node(
                x_ind, y_ind, yaw_ind, direction, node, node.cost + step_cost,
                steering, new_run_length
            ))
        
    return next_nodes

# =========================================================
#  בדיקת התנגשויות (כולל אינטרפולציה)
# =========================================================
# def check_collision(node, obstacle_set, xy_res):
#     curr_x = node.x_ind * xy_res
#     curr_y = node.y_ind * xy_res
    
#     # גבולות המגרש
#     if curr_x <= 0.1 or curr_x >= 4.9 or curr_y <= 0.1 or curr_y >= 4.9:
#         return False

#     # בדיקת רדיוס מהירה של נקודת הקצה
#     for (ox_ind, oy_ind) in obstacle_set:
#         dist = math.hypot(curr_x - ox_ind * xy_res, curr_y - oy_ind * xy_res)
#         if dist <= ROBOT_RADIUS:
#             return False
            
#     # אינטרפולציה למניעת מעבר דרך קירות דקים (Tunneling)
#     if node.p_node:
#         prev_x = node.p_node.x_ind * xy_res
#         prev_y = node.p_node.y_ind * xy_res
#         for i in range(1, 4): # בדיקת 3 נקודות ביניים
#             t = i / 4.0
#             ix = prev_x + (curr_x - prev_x) * t
#             iy = prev_y + (curr_y - prev_y) * t
#             for (ox_ind, oy_ind) in obstacle_set:
#                 if math.hypot(ix - ox_ind * xy_res, iy - oy_ind * xy_res) <= ROBOT_RADIUS:
#                     return False

#     return True

def build_spatial_index(obstacle_set):
    # Uses spatial bucketing to limit collision checks to nearby obstacles, significantly speeding up A* search on large maps.
    index = {}
    for (ox, oy) in obstacle_set:
        key = (ox // BUCKET_CELLS, oy // BUCKET_CELLS)
        index.setdefault(key, []).append((ox, oy))
    return index

def check_collision(node, obstacle_set, xy_res, spatial_index):
    curr_x = node.x_ind * xy_res
    curr_y = node.y_ind * xy_res

    # 1. גבולות המגרש (configurable geofence, see set_arena_bounds)
    if curr_x <= ARENA_MIN or curr_x >= ARENA_MAX or curr_y <= ARENA_MIN or curr_y >= ARENA_MAX:
        return False

    # 2. בדיקה שהמשבצת הנוכחית אינה קיר
    if (node.x_ind, node.y_ind) in obstacle_set:
        return False

    # 3. מניעת חיתוך קירות באלכסון (Tunneling Fix)
    if node.p_node:
        px_ind = node.p_node.x_ind
        py_ind = node.p_node.y_ind
        cx_ind = node.x_ind
        cy_ind = node.y_ind

        # מניעת חיתוך קירות באלכסון (Tunneling Fix)
        if px_ind != cx_ind and py_ind != cy_ind:
            # בודקים את שתי המשבצות שיוצרות את ה"פינה" של הקיר
            if (px_ind, cy_ind) in obstacle_set or (cx_ind, py_ind) in obstacle_set:
                return False # חוסם!

    # 4. בדיקת רדיוס רובוט מול קירות קרובים בלבד (spatial index, ראה build_spatial_index)
    bx, by = node.x_ind // BUCKET_CELLS, node.y_ind // BUCKET_CELLS
    for dbx in (-1, 0, 1):
        for dby in (-1, 0, 1):
            for (ox_ind, oy_ind) in spatial_index.get((bx + dbx, by + dby), ()):
                dist = math.hypot(curr_x - ox_ind * xy_res, curr_y - oy_ind * xy_res)
                if dist <= PLANNING_CLEARANCE:
                    return False
    return True

def extract_path(end_node, xy_res, yaw_res):
    rx, ry, rt, rd = [], [], [], [] 
    node = end_node
    while node is not None:
        rx.append(node.x_ind * xy_res)
        ry.append(node.y_ind * xy_res)
        rt.append(node.yaw_ind * yaw_res)
        rd.append(node.direction)
        node = node.p_node
    
    rx.reverse()
    ry.reverse()
    rt.reverse()
    rd.reverse()
    return rx, ry, rt, rd