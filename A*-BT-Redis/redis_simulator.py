# import redis
# import json
# import time
# import random

# # מתחברים לשרת הרדיס המקומי
# # decode_responses=True הופך את המידע שחוזר לטקסט רגיל במקום לביטים
# r = redis.Redis(host='localhost', port=6379, db=0, decode_responses=True)

# print("🚀 סימולטור הרדיס מתחיל לשדר...")

# while True:
#     # 1. יצירת נתונים פיקטיביים לאויבים (רשימה של מילונים)
#     enemies_data = [
#         {"timestamp": time.time(), "x": random.uniform(1.0, 5.0), "y": random.uniform(1.0, 5.0)}
#     ]
#     # 2. יצירת נתונים פיקטיביים לחברי צוות
#     # נגריל באופן אקראי האם רובוט_3 צריך עזרה כדי לראות איך העץ מגיב
#     team_data = {
#         "robot_2": {"x": 4.0, "y": 4.5, "needs_help": False},
#         "robot_3": {"x": 3.0, "y": 2.0, "needs_help": random.choice([True, False])} 
#     }

#     # 3. המרה ל-JSON ושליחה לערוצים
#     r.publish('/detected_enemies', json.dumps(enemies_data))
#     r.publish('/team/positions', json.dumps(team_data))

#     print(f"📡 שודר עדכון: אויבים={len(enemies_data)}, חברי צוות={len(team_data)}")

#     time.sleep(1) # משדר פעם בשנייה כדי לא להציף



import redis
import json
import time
import math
import random

# חיבור לרדיס (שים לב לוודא שהפורט ושרת הרדיס שלך רצים)
r = redis.Redis(host='localhost', port=6379, db=0, decode_responses=True)

print("🚀 סימולטור תנועה מציאותי מתחיל לשדר...")

# ==========================================
# 1. הגדרת מיקומי התחלה
# ==========================================
enemy_x, enemy_y = 1.0, 1.0
r2_x, r2_y = 4.0, 4.5
r3_x, r3_y = 3.0, 2.0

angle = 0.0 # זווית שתעזור לנו לייצר תנועה חלקה

while True:
    # ==========================================
    # 2. חישוב "הצעד" הבא (מייצר מסלול אמיתי)
    # ==========================================
    angle += 0.2  # מקדמים את הזווית
    
    # האויב עושה תנועת סיור מעגלית רחבה
    enemy_x = 3.0 + math.cos(angle) * 2.0
    enemy_y = 3.0 + math.sin(angle) * 2.0
    
    # רובוט 2 עושה תנועה חלקה הלוך-חזור על ציר ה-X (פטרול)
    r2_x = 4.0 + math.sin(angle * 0.5) * 1.5
    
    # רובוט 3 סתם רועד קצת במקום (מתקן מיקום)
    r3_x += random.uniform(-0.05, 0.05)
    r3_y += random.uniform(-0.05, 0.05)

    # נגריל עזרה לרובוט 3 (סיכוי קטן יותר, כדי שלא יצעק "הצילו" כל שנייה)
    needs_help_chance = random.choice([True, False, False, False, False])

    # ==========================================
    # 3. אריזה לפורמטים המדויקים שביקשת
    # ==========================================
    
    # enemies_data = [
    #     {"timestamp": time.time(), "x": enemy_x, "y": enemy_y}
    # ]

    enemies_data = [
        {"timestamp": time.time(), "x": 4.5, "y": 4.0}
    ]
    # חברי צוות - כל רובוט אורז את עצמו למילון עצמאי! (בלי מפתח חיצוני)
    # r2_data = {"robot_id": "robot_2", "x": r2_x, "y": r2_y, "needs_help": False}
    # r3_data = {"robot_id": "robot_3", "x": r3_x, "y": r3_y, "needs_help": needs_help_chance}

    # ==========================================
    # 4. שידור לרדיס
    # ==========================================
    r.publish('/detected_enemies', json.dumps(enemies_data))
    
    # משדרים כל רובוט בנפרד, כאילו שני רובוטים שונים צעקו ברשת
    # r.publish('/team/positions', json.dumps(r2_data))
    # r.publish('/team/positions', json.dumps(r3_data))

    # הדפסה יפה כדי שתראה את התנועה בטרמינל
    print(f"📡 שודר: אויב(X:{enemy_x:.1f}, Y:{enemy_y:.1f}) | רובוט_2(X:{r2_x:.1f}, Y:{r2_y:.1f})")

    # מחכה שנייה אחת כדי לא להציף את הרשת (אפשר להוריד ל-0.1 אם אתה רוצה תנועה מהירה יותר)
    time.sleep(0.4)