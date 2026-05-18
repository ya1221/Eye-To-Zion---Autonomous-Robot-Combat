import math
import json
import numpy as np
import time

#from A_planner import XY_RESOLUTION

R_ENEMY = 0.5  # meters
R_TEAMMATE = 0.3  # meters
SAME_ENEMY_THRESHOLD = 0.5
XY_RESOLUTION = 0.1 
danger_dict = {}  # {(x_index, y_index): timestamp}

def get_latest_world_state(p, current_enemies_list, current_teammates_dict, static_obstacles):
    global danger_dict
    
    enemies_list, teammates_dict = check_redis_updates(p, current_enemies_list, current_teammates_dict)
    
    danger_dict = get_danger_dict(danger_dict, enemies_list, static_obstacles)

    danger_dict = delete_old_danger_squares(danger_dict)

    teammates_aura_set = get_teammates_aura_set(teammates_dict)
    
    return danger_dict, teammates_aura_set, enemies_list, teammates_dict

def check_redis_updates(p, enemies_list, teammates_dict):
    msg = p.get_message()
    
    while msg is not None:
        if msg["type"] == "message":
            channel_name = msg["channel"].decode('utf-8')
            parsed_data = json.loads(msg["data"])
            
            if channel_name == "/detected_enemies":
                current_time = time.time()
                
                # עוברים על כל הזיהויים החדשים שהגיעו הרגע מהמצלמה
                for new_enemy in parsed_data:
                    # מוודאים שיש חותמת זמן
                    if "timestamp" not in new_enemy:
                        new_enemy["timestamp"] = current_time
                        
                    found_match = False
                    
                    # משווים מול האויבים שכבר יש לנו בזיכרון
                    for existing_enemy in enemies_list:
                        # חישוב המרחק בין הזיהוי החדש לאויב הקיים
                        dist = distance((new_enemy["x"], new_enemy["y"]), (existing_enemy["x"], existing_enemy["y"]))
                        
                        # אם המרחק קטן מהסף - זה אותו אויב!
                        if dist <= SAME_ENEMY_THRESHOLD:
                            # אנחנו לא מוסיפים אויב חדש, אלא *מעדכנים* את המיקום והזמן של הקיים
                            existing_enemy["x"] = new_enemy["x"]
                            existing_enemy["y"] = new_enemy["y"]
                            existing_enemy["timestamp"] = new_enemy["timestamp"]
                            
                            found_match = True
                            break # מצאנו לו שידוך, אפשר לעבור לזיהוי הבא מהמצלמה
                            
                    # אם עברנו על כל הזיכרון שלנו וזה לא קרוב לאף אחד - יש פה באמת אויב חדש!
                    if not found_match:
                        enemies_list.append(new_enemy)

            elif channel_name == "/team/positions":
                robot_id = parsed_data.get("robot_id")
                if robot_id:
                    teammates_dict[robot_id] = parsed_data

        msg = p.get_message()
        
    # --- מנגנון ניקוי הזיכרון (Pruning) נשאר אותו דבר ---
    current_time = time.time()
    MEMORY_TIMEOUT = 2.0 # נשכח אויב שלא ראינו 2 שניות
    
    # משאירים רק את מי שהתעדכן לאחרונה (הטריים)
    enemies_list[:] = [
        enemy for enemy in enemies_list 
        if current_time - enemy.get("timestamp", current_time) < MEMORY_TIMEOUT
    ]
    
    return enemies_list, teammates_dict

def get_danger_dict(danger_dict, enemies_list, static_obstacles):
    for enemy_data in enemies_list:
        enemy_x = int(enemy_data["x"] / XY_RESOLUTION)
        enemy_y = int(enemy_data["y"] / XY_RESOLUTION)
        r_enemy_squares = int(R_ENEMY / XY_RESOLUTION)

        for x in range(enemy_x - r_enemy_squares, enemy_x + r_enemy_squares + 1):
            for y in range(enemy_y - r_enemy_squares, enemy_y + r_enemy_squares + 1):
                if (distance((enemy_x, enemy_y), (x, y)) <= r_enemy_squares # circle check
                    and line_of_sight_clear((enemy_x, enemy_y), (x, y), static_obstacles)): # line of sight check
                    danger_dict[(x, y)] = enemy_data["timestamp"]
    
    return danger_dict

def get_current_time():
    return time.time()

def delete_old_danger_squares(danger_dict):
    current_time = get_current_time() 
    for square in list(danger_dict.keys()):
        if current_time - danger_dict[square] >= 15:
            danger_dict.pop(square)

    return danger_dict

def get_teammates_aura_set(teammates_dict):
    teammates_aura_set = set()
    for teammate_data in teammates_dict.values():
        
        # if not isinstance(teammate_data, dict):
        #     print(f"ERROR: Expected dict for teammate, got: {type(teammate_data)} -> {teammate_data}")
        #     continue

        teammate_x = int(teammate_data["x"] / XY_RESOLUTION)
        teammate_y = int(teammate_data["y"] / XY_RESOLUTION)
        r_teammate_squares = int(R_TEAMMATE / XY_RESOLUTION)    

        for x in range(teammate_x - r_teammate_squares, teammate_x + r_teammate_squares + 1):
            for y in range(teammate_y - r_teammate_squares, teammate_y + r_teammate_squares + 1):
                    if distance((teammate_x, teammate_y), (x, y)) <= r_teammate_squares: # circle check
                        teammates_aura_set.add((x, y))
    
    return teammates_aura_set

def line_of_sight_clear(start, end, walls_set):
    # חישוב המרחק הגיאומטרי
    dist = distance(start, end)
    
    # יצירת מספיק נקודות על הקו כדי לא לפספס אף משבצת (רזולוציה)
    num_points = int(dist * 10) + 1 
    
    x_points = np.linspace(start[0], end[0], num_points)
    y_points = np.linspace(start[1], end[1], num_points)
    
    # מעבר על כל הנקודות לאורך הקו
    for x, y in zip(x_points, y_points):
        # עיגול למספרים שלמים כדי לקבל את אינדקס המשבצת ברשת
        grid_x, grid_y = round(x), round(y)
        
        # אם המשבצת הנוכחית היא קיר - קו הראייה נחסם
        if (grid_x, grid_y) in walls_set:
            return False 
            
    # אם סיימנו את הלולאה ולא פגענו בקיר - הדרך פנויה
    return True

def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])