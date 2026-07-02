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
# nav2's real keepout is bigger than this: footprint [0.18,0.13]'s diagonal
# reach (~0.22m) plus costmap inflation_radius (0.22m, nav2.yaml) is a
# worst-case theoretical bound of ~0.44m (corner-on approach to a wall,
# treating the soft inflation cost gradient as a hard cutoff - it isn't
# really one). A* using only ROBOT_RADIUS could approve a goal/path nav2
# then can never physically execute (confirmed directly: goal (1.3, 1.7)
# had 0.25-0.44m clearance - passed the old ROBOT_RADIUS-only check, but
# the robot stalled ~0.25-0.3m short of it every attempt and got
# progress-checker-aborted). PLANNING_CLEARANCE is what check_collision
# actually enforces, so A* only ever proposes goals/paths nav2 can
# genuinely reach.
#
# 0.44 (the theoretical worst case above) is stricter than necessary in
# practice - empirically binary-searched down: 0.35 failed cleanly (2/2
# aborts) on that same (1.3, 1.7) goal, 0.40 succeeded cleanly (dozens of
# consecutive successes on both (1.3, 1.7) and a known-good goal). Kept
# at 0.40 rather than narrowing further - the gap to 0.35 is not large
# enough to be worth more search time, and a real margin below the
# theoretical 0.44 bound is reassuring rather than concerning.
PLANNING_CLEARANCE = 0.40
BUCKET_CELLS = math.ceil(PLANNING_CLEARANCE / XY_RESOLUTION) + 1  # spatial-index bucket size for check_collision

# === פרמטרי עלויות (Costs) ===
H_WEIGHT = 1.2        # משקל ההיוריסטיקה (Weighted A*) - מאוזן למניעת תקיעה
# Minimum consecutive steps (0.6m at MOVE_STEP=0.2) a direction run must
# last before another gear switch is "free" - see calc_motion's
# switching/run_length handling. Confirmed directly: without this, A*'s
# raw step-by-step direction output sometimes alternated every step,
# producing 1-2 point same-direction runs that Nav2BaseAction then had to
# send to FollowPath as their own degenerate segments.
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
        # Consecutive steps taken in the current direction. Used to
        # penalize switching gear again too soon (see MIN_RUN_STEPS in
        # calc_motion) - without this, the search could produce paths
        # that flip direction every step, which Nav2BaseAction then has
        # to split into degenerate 1-2 point FollowPath segments that
        # MPPI can't track sensibly.
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
        # Require the final approach step to be forward: reverse+steer near
        # the goal is where Gazebo ODE friction makes the robot stall (see
        # etz-open-issues memory). The goal's heading param is otherwise
        # unused by this search (termination/heuristic are x,y-only), so it
        # can't bias this - direction must be checked on the terminal node
        # itself instead.
        if dist_to_goal <= 0.1 and current.direction == 1:
            rx, ry, rt, rd = extract_path(current, xy_resolution, yaw_resolution)
            # Stop expanding once within 0.1m of goal (one grid cell) and
            # snap the last waypoint to the exact goal coordinate so
            # FollowPath's final pose is the real target. Tight radius keeps
            # the path geometrically complete: anything larger (e.g. 0.3m)
            # causes A* to return a trivial 2-step [current→goal] path when
            # the robot is near the goal with a large heading mismatch,
            # which MPPI can't execute in tight spaces.
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
    # Forward-only was tried and reverted: it made MPPI execution cleaner
    # (no more reverse segments to discard going into nav_msgs/Path), but
    # broke planning outright in tight spaces - confirmed directly, A*
    # returned "failed to find a path" once obstacle_set had real walls
    # in a narrow spot, since a forward-only Ackermann turn needs more
    # room than a real car needing to reverse out of a tight spot. Kept
    # reverse in the search; Nav2BaseAction now splits the path into
    # per-direction segments instead of sending one mixed-direction path.
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
                # Switched again before finishing a minimum-length leg -
                # escalating penalty per step short, on top of the flat
                # switch_gear_cost above, so the search prefers fewer,
                # longer same-direction runs over flip-flopping.
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
    # check_collision's radius check used to scan the entire obstacle_set
    # per candidate node - fine early in a run (~200 cells) but scaled
    # linearly with how much slam_toolbox has mapped, confirmed directly
    # to blow up to ~300s per A* search once the map grew past ~1600
    # occupied cells. Bucketing lets check_collision only look at the
    # handful of obstacles actually near the candidate point.
    index = {}
    for (ox, oy) in obstacle_set:
        key = (ox // BUCKET_CELLS, oy // BUCKET_CELLS)
        index.setdefault(key, []).append((ox, oy))
    return index

def check_collision(node, obstacle_set, xy_res, spatial_index):
    curr_x = node.x_ind * xy_res
    curr_y = node.y_ind * xy_res

    # 1. גבולות המגרש
    if curr_x <= 0.1 or curr_x >= 4.9 or curr_y <= 0.1 or curr_y >= 4.9:
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