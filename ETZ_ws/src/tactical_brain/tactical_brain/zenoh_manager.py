import math
import json
import time
import queue

# קבועים
SAME_ENEMY_THRESHOLD = 0.5

# הגדר את ה-ID של הרובוט שעליו רץ הקוד הזה (בהנחה שעוגנים הם 0-3)
MY_ARUCO_ID = 4

# Pure Zenoh-message decoding - no ROS imports, no global state. Owned and
# called by zenoh_node.py, which is the only thing that knows about Zenoh
# sessions/subscriptions and ROS publishers.
def check_zenoh_updates(msg_queue, enemies_list, teammates_dict):
    current_time = time.time()
    self_pose = None

    while not msg_queue.empty():
        try:
            msg = msg_queue.get_nowait()
        except queue.Empty:
            break

        channel_name = msg["channel"]
        try:
            parsed_data = json.loads(msg["data"])
        except json.JSONDecodeError:
            continue

        # השתמשנו ב-"in" במקום ב-"==" כדי לתמוך בקידומות כמו "team_blue/detected_enemies"
        if "detected_enemies" in channel_name:
            for new_enemy in parsed_data:
                if "timestamp" not in new_enemy:
                    new_enemy["timestamp"] = current_time

                found_match = False
                for existing_enemy in enemies_list:
                    dist = distance((new_enemy["x"], new_enemy["y"]), (existing_enemy["x"], existing_enemy["y"]))
                    if dist <= SAME_ENEMY_THRESHOLD:
                        existing_enemy["x"] = new_enemy["x"]
                        existing_enemy["y"] = new_enemy["y"]
                        existing_enemy["timestamp"] = new_enemy["timestamp"]
                        found_match = True
                        break

                if not found_match:
                    enemies_list.append(new_enemy)

        elif channel_name.endswith("/team_positions") or "/team/positions" in channel_name:
            robot_id = parsed_data.get("robot_id")
            if robot_id:
                # כאן מתעדכנים כל חברי הצוות באופן בלעדי
                teammates_dict[robot_id] = parsed_data

        # הטופיק החדש של מצלמת הארוקו: teams/team_{team_idx}/positions - מערך JSON
        # של כל הרובוטים בקבוצה. בדיקת startswith/endswith (ולא "in") כדי לא
        # להתבלבל עם הערוץ הישן "team_positions" שגם מכיל את המילה positions.
        elif channel_name.startswith("teams/team_") and channel_name.endswith("/positions"):
            # חיפוש עצמי בלבד - הרובוט שעליו רץ הקוד הזה
            for bot in parsed_data:
                if bot.get("id") == MY_ARUCO_ID:
                    bot_x_m = bot.get("x", 0) / 100.0
                    bot_y_m = bot.get("y", 0) / 100.0
                    bot_angle_rad = math.radians(bot.get("angle", 0.0))
                    self_pose = (bot_x_m, bot_y_m, bot_angle_rad)
                    break

    MEMORY_TIMEOUT = 2.0
    enemies_list[:] = [
        enemy for enemy in enemies_list
        if current_time - enemy.get("timestamp", current_time) < MEMORY_TIMEOUT
    ]

    return enemies_list, teammates_dict, self_pose


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])
