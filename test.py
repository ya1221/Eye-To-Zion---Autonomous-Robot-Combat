import zenoh
import json
import time
import math
import random
import threading

# הגדרות הרשת
TEAM_PREFIX = "team_blue"
MY_ARUCO_ID = 4      # ה-ID של הרובוט הראשי שלך (זה שמריץ את המוח)
TEAMMATE_ID = 5      # ה-ID של הרובוט החבר שמדמה הסימולטור הזה

print("========================================")
print("  Zenoh Fleet Simulator - ONLINE  ")
print("========================================")
print("Connecting to Zenoh network...")

# אתחול Zenoh
# אתחול Zenoh כשרת שמקשיב לחיבורים נכנסים
conf = zenoh.Config()
conf.insert_json5("listen/endpoints", '["tcp/100.91.30.13:7447"]')
session = zenoh.open(conf)

def simulate_aruco_camera():
    """
    סימולציה של מצלמת התקרה. משדרת את המיקום של כולם במגרש בסנטימטרים ומעלות
    """
    publisher = session.declare_publisher(f'{TEAM_PREFIX}/fleet_positions')
    
    # מיקום התחלתי שזז לאט
    my_x, my_y, my_angle = 200, 200, 0   # 2x2 מטר
    team_x, team_y, team_angle = 150, 300, 45 # 1.5x3.0 מטר
    
    while True:
        # הזזה רנדומלית קטנה כדי לדמות תנועה טבעית
        my_x += random.randint(-5, 5)
        my_y += random.randint(-5, 5)
        my_angle = (my_angle + random.randint(-2, 2)) % 360
        
        team_x += random.randint(-2, 2)
        team_y += random.randint(-2, 2)
        team_angle = (team_angle + random.randint(-5, 5)) % 360
        
        fleet_data = [
            {"id": MY_ARUCO_ID, "x": my_x, "y": my_y, "angle": my_angle},
            {"id": TEAMMATE_ID, "x": team_x, "y": team_y, "angle": team_angle}
        ]
        
        publisher.put(json.dumps(fleet_data))
        time.sleep(1.0) # משדר כל שנייה

def simulate_yolo_camera():
    """
    סימולציה של מצלמת הרובוט. מדי פעם משדרת זיהוי אויב
    """
    publisher = session.declare_publisher(f'{TEAM_PREFIX}/detected_enemies')
    
    while True:
        # נדמה אויב שמופיע כל כמה שניות
        time.sleep(random.uniform(2.0, 5.0))
        
        # האויב "צץ" במרחק של מטר מהרובוט הראשי
        enemy_data = [{
            "id": 99, 
            "x": 2.0 + random.uniform(-0.5, 0.5), # קואורדינטות קרטזיות גלובליות
            "y": 2.0 + random.uniform(0.5, 1.5), 
            "timestamp": time.time()
        }]
        
        print(f"[YOLO] Spotted enemy at X:{enemy_data[0]['x']:.2f}, Y:{enemy_data[0]['y']:.2f}")
        publisher.put(json.dumps(enemy_data))

def simulate_teammate_comms():
    """
    סימולציה של חבר לצוות. משדר את המיקום שלו ולפעמים מבקש עזרה
    """
    publisher = session.declare_publisher(f'{TEAM_PREFIX}/team_positions')
    
    while True:
        time.sleep(2.0)
        
        # מדי פעם החבר נקלע למצוקה ומבקש עזרה!
        needs_help = random.choice([False, False, False, True]) # 25% סיכוי לעזרה
        
        teammate_data = {
            "robot_id": TEAMMATE_ID,
            "x": 1.5,
            "y": 3.0,
            "needs_help": needs_help
        }
        
        if needs_help:
            print(f"[TEAMMATE {TEAMMATE_ID}] Sending distress signal (needs_help=True)!")
            
        publisher.put(json.dumps(teammate_data))

# הפעלת הסימולציות כתהליכים מקבילים (Threads)
threads = [
    threading.Thread(target=simulate_aruco_camera, daemon=True),
    threading.Thread(target=simulate_yolo_camera, daemon=True),
    threading.Thread(target=simulate_teammate_comms, daemon=True)
]

for t in threads:
    t.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nSimulator shut down.")