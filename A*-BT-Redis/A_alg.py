import matplotlib.pyplot as plt
import numpy as np
import math
import time
import redis
import json
import A_planner
import redis_manager # מייבאים את הפונקציות המעולות שלך

XY_RESOLUTION = 0.1
LIDAR_RANGE = 3.0
LIDAR_ANGLE = 360
MAX_DISTANCE = 1.5
# ==========================================
# פונקציות עזר וסביבה
# ==========================================
def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def get_line_points(start, end):
    dist = distance(start, end)
    # שינוי קריטי: מכפילים פי 100 כדי ליצור קו סופר-צפוף (ללא חורים)
    num_points = int(dist * 100) + 1
    x_points = np.linspace(start[0], end[0], num_points)
    y_points = np.linspace(start[1], end[1], num_points)
    return x_points, y_points

def get_obstacle_set():
    obstacle_set = set()
    ox, oy = generate_walls_map()
    for x, y in zip(ox, oy):
        # שינוי קריטי: שימוש ב-round במקום רק int כדי למנוע חיתוך (truncation)
        # זה מבטיח שהקיר יישב בדיוק במרכז המשבצת הקרובה ביותר
        grid_x = int(round(x / XY_RESOLUTION))
        grid_y = int(round(y / XY_RESOLUTION))
        obstacle_set.add((grid_x, grid_y))
    return obstacle_set

def generate_walls_map():
    ox, oy = [], []
    walls = [
        ((0, 0), (5, 0)),
        ((0, 0), (0, 5)),
        ((5, 0), (5, 5)),
        ((0, 5), (5, 5)),
        ((1, 1), (1, 4)),
        ((1, 3), (4, 3)),
        ((3, 1), (3, 2)),
        ((3.5, 3), (4, 4)),
        ((0, 4), (1, 4)),
        #((4.0, 4.0), (5.0, 4.0))
    ]
    for wall in walls:
        x_points, y_points = get_line_points(*wall)
        ox.extend(x_points)
        oy.extend(y_points)
    return ox, oy

# def get_obstacle_set():
#     obstacle_set = set()
#     ox, oy = generate_walls_map()
#     for x, y in zip(ox, oy):
#         obstacle_set.add((int(x / XY_RESOLUTION), int(y / XY_RESOLUTION)))
#     return obstacle_set

def scan_env(robot_pos, real_obstacle_set):
    detected = []
    for angle in np.arange(0, LIDAR_ANGLE, 1):
        angle_rad = np.deg2rad(angle)
        for r in np.arange(0, LIDAR_RANGE, XY_RESOLUTION):
            cx = robot_pos[0] + r * math.cos(angle_rad)
            cy = robot_pos[1] + r * math.sin(angle_rad)
            idx = (int(cx / XY_RESOLUTION), int(cy / XY_RESOLUTION))
            if idx in real_obstacle_set:
                detected.append(idx)
                break
    return detected

def assign_robot_id(redis_conn):
    new_number = redis_conn.incr("global_robot_counter")
    return f"robot_{new_number}"

# ==========================================
# הלולאה הראשית - הסימולציה
# ==========================================
def run_simulation(start_pos, goal_pos):
    print("Simulation Started - Tactical Combat Mode...")

    real_obstacle_set = get_obstacle_set()
    detected_obstacles = set()

    traveled_path = [start_pos]
    failed_paths = []
    robot_pos = start_pos
    current_path_obj = None 
    current_path_index = 0  
    need_replanning = True 
    
    robot_state = "NAVIGATING" 
    fig = plt.figure(figsize=(10, 10))

    r = redis.Redis(host='localhost', port=6379, decode_responses=False)

    MY_ROBOT_ID = assign_robot_id(r)

    p = r.pubsub()
    
    p.subscribe('/detected_enemies', '/team/positions')
    
    # מילונים ריקים שיתעדכנו מהרדיס לאורך זמן
    enemies_list = []
    teammates_dict = {}

    while True:
        loop_start_time = time.time()
        current_t = loop_start_time

        danger_dict, teammates_aura_set, enemies_list, teammates_dict = redis_manager.get_latest_world_state(
            p, enemies_list, teammates_dict, real_obstacle_set
        )

        # teammates_dict = {(4.5, 2.0): current_t} 
        # TEAMMATE_RADIUS = 0.5
        # teammates_aura_set = set()
        
        # for (tx, ty), t in teammates_dict.items():
        #     for ix in np.arange(tx - TEAMMATE_RADIUS, tx + TEAMMATE_RADIUS, XY_RESOLUTION):
        #         for iy in np.arange(ty - TEAMMATE_RADIUS, ty + TEAMMATE_RADIUS, XY_RESOLUTION):
        #             if math.hypot(ix - tx, iy - ty) <= TEAMMATE_RADIUS:
        #                 teammates_aura_set.add((int(ix / XY_RESOLUTION), int(iy / XY_RESOLUTION)))


        #danger_dict = {}
        
        # ==================================================
        # 1. סינון מטרות וזיהוי המטרה הגלויה הקרובה ביותר
        # ==================================================
        closest_enemy_pos = None
        min_dist = float('inf')
        
        # אינדקסים של הרובוט לבדיקת קו ראייה
        rx_ind = int(robot_pos[0] / XY_RESOLUTION)
        ry_ind = int(robot_pos[1] / XY_RESOLUTION)

        for enemy in enemies_list:
            ex = enemy["x"]
            ey = enemy["y"]
            
            d = math.hypot(robot_pos[0] - ex, robot_pos[1] - ey) # מרחק אווירי
            if d > MAX_DISTANCE:
                continue

            if d < min_dist:
                ex_ind = int(ex / XY_RESOLUTION)
                ey_ind = int(ey / XY_RESOLUTION)
                
                # מוודאים שיש אליו קו ראייה נקי
                has_los = redis_manager.line_of_sight_clear((rx_ind, ry_ind), (ex_ind, ey_ind), real_obstacle_set)
                
                if has_los:
                    min_dist = d
                    closest_enemy_pos = (ex, ey)

        # ==================================================
        # 2. ניווט ותכנון מסלול
        # ==================================================
        if distance(robot_pos, goal_pos) <= 0.5:
            print("Goal reached!")
            break

        if need_replanning and robot_state == "NAVIGATING":
            result = A_planner.calc_hybrid_a_star(
                robot_pos, goal_pos, detected_obstacles,
                XY_RESOLUTION, A_planner.YAW_RESOLUTION,
                danger_dict, teammates_aura_set
            )

            if not result:
                print("Critical: Could not find a path!")
                robot_state = "STUCK"
                need_replanning = False
                continue
            
            current_path_obj, path_cost = result
            current_path_index = 0
            need_replanning = False
            
            # --- הלוגיקה החדשה: האם המסלול דורך על סכנה? ---
            path_crosses_enemy = False
            x_path, y_path, _, _ = current_path_obj
            
            for px, py in zip(x_path, y_path):
                p_ind = (int(round(px / XY_RESOLUTION)), int(round(py / XY_RESOLUTION)))
                if p_ind in danger_dict:
                    path_crosses_enemy = True
                    break
            
            if path_crosses_enemy:
                print("\n==================================")
                print("Target identified on path! ENTERING INTERCEPTING MODE ⚔️")
                print("==================================\n")
                robot_state = "INTERCEPTING"

        # ==================================================
        # 3. תנועה, בלימה טקטית וסריקת לידאר
        # ==================================================
        if robot_state in ["NAVIGATING", "INTERCEPTING"]:
            x_path, y_path, theta_path, directions = current_path_obj
            
            if current_path_index < len(x_path) - 1:
                current_path_index += 1
                next_x = x_path[current_path_index]
                next_y = y_path[current_path_index]
                next_theta = theta_path[current_path_index]
                robot_pos = (next_x, next_y, next_theta)
                traveled_path.append(robot_pos)
                
                # --- מעבר מ-INTERCEPTING ל-COMBAT ---
                if robot_state == "INTERCEPTING" and closest_enemy_pos:
                    if min_dist <= 0.8: # טווח עצירה וירי
                        print(f"\n*** Target Locked at {closest_enemy_pos}! LOS verified. Braking for Combat! ***\n")
                        robot_state = "COMBAT"
            else:
                need_replanning = True

            # --- סריקת לידאר תוך כדי תנועה (תיקון ה-FPS והחסימות) ---
            new_obs = scan_env(robot_pos, real_obstacle_set)
            if new_obs:
                truly_new_obs = [obs for obs in new_obs if obs not in detected_obstacles]
                detected_obstacles.update(new_obs)
                
                if truly_new_obs:
                    is_blocked = False
                    horizon = min(len(x_path), current_path_index + 15)
                    for k in range(current_path_index, horizon):
                        fx, fy = x_path[k], y_path[k]
                        for ox_ind, oy_ind in truly_new_obs: 
                            ox_m = ox_ind * XY_RESOLUTION
                            oy_m = oy_ind * XY_RESOLUTION
                            # בדיקת רדיוס מדויקת ללא ה-+0.1 שעשה צרות
                            if math.hypot(fx - ox_m, fy - oy_m) <= A_planner.ROBOT_RADIUS:
                                is_blocked = True
                                break
                        if is_blocked: break
                    
                    if is_blocked:
                        # print("New wall detected on path! Replanning...")
                        failed_paths.append(current_path_obj)
                        need_replanning = True

        elif robot_state == "COMBAT":
            # הרובוט עומד במקום
            pass
        
        # ==================================================
        # שליחה ברדיס
        # ==================================================
        my_data = {
            "robot_id": MY_ROBOT_ID,
            "x": robot_pos[0],
            "y": robot_pos[1],
        }
        
        # # שידור לערוץ המטרות/אויבים (כדי שהטרמינל השני יזהה אותו כאויב ויתקוף!)
        # #r.publish('/detected_enemies', json.dumps(my_data))
        
        # # או לחלופין, אם אתה רוצה שהם יזהו אחד את השני כחברים לצוות:
      #  r.publish('/team/positions', json.dumps(my_data))


        # ==================================================
        # 4. ציור וביצועים
        # ==================================================
        draw_simulation(robot_pos, goal_pos, current_path_obj,
                        real_obstacle_set, detected_obstacles,
                        traveled_path, failed_paths, danger_dict, closest_enemy_pos, teammates_aura_set)
        
        #loop_time = time.time() - loop_start_time
        #fps = 1.0 / loop_time if loop_time > 0 else 999
        # הדפסת ביצועים מסודרת - שים לב לזה בקונסול!
        # print(f"FPS: {fps:.1f} | State: {robot_state}")

# ==========================================
# פונקציית הציור המעודכנת
# ==========================================
def draw_simulation(robot_pos, goal_pos, current_path,
                    real_obstacles, detected_obstacles,
                    traveled_path, failed_paths, danger_dict,
                    closest_enemy_pos, teammates_aura_set):
    
    plt.cla()
    
    # 1. ציור אויבים (אדום) - למרות שבטסט הזה זה יהיה ריק
    if danger_dict:
        ex = [x * XY_RESOLUTION for x, y in danger_dict.keys()]
        ey = [y * XY_RESOLUTION for x, y in danger_dict.keys()]
        plt.plot(ex, ey, "sr", alpha=0.3, markersize=8)
    
    if closest_enemy_pos:
        plt.plot([closest_enemy_pos[0]], [closest_enemy_pos[1]], "Xr", markersize=12)

    # 2. ציור חברי צוות (תכלת)
    if teammates_aura_set:
        tx = [x * XY_RESOLUTION for x, y in teammates_aura_set]
        ty = [y * XY_RESOLUTION for x, y in teammates_aura_set]
        plt.plot(tx, ty, "sc", alpha=0.3, markersize=8) # הילה תכלת
        plt.plot([3.0], [2.5], "Xb", markersize=10)     # סמן איקס כחול למרכז חבר הצוות
    # קירות ולידאר
    ox = [x * XY_RESOLUTION for x, y in real_obstacles]
    oy = [y * XY_RESOLUTION for x, y in real_obstacles]
    plt.plot(ox, oy, ".k", markersize=1)

    if detected_obstacles:
        dx = [x * XY_RESOLUTION for x, y in detected_obstacles]
        dy = [y * XY_RESOLUTION for x, y in detected_obstacles]
        plt.plot(dx, dy, ".b", markersize=4) 

    for path in failed_paths:
        px, py, _, _ = path
        plt.plot(px, py, "--", color="gray", linewidth=0.5)

    if current_path:
        px, py, _, pd = current_path
        for i in range(len(px) - 1):
            color = 'g' if pd[i] == 1 else 'm'
            lw = 2 if pd[i] == 1 else 3
            plt.plot([px[i], px[i+1]], [py[i], py[i+1]], color=color, linewidth=lw)

    if traveled_path:
        tx = [p[0] for p in traveled_path]
        ty = [p[1] for p in traveled_path]
        plt.plot(tx, ty, "-b", linewidth=1)

    # ציור הרובוט
    plt.arrow(robot_pos[0], robot_pos[1],
              0.4 * math.cos(robot_pos[2]),
              0.4 * math.sin(robot_pos[2]),
              head_width=0.15)

    plt.plot(goal_pos[0], goal_pos[1], "xg", markersize=10)
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.001)

if __name__ == "__main__":
    # מבחן המטווח - רובוט ויעד על קו 2.5
    robot_pos = (2.5, 0.5, math.radians(90))
    goal_pos = (2.5, 4.5, 0.0)
    run_simulation(robot_pos, goal_pos)