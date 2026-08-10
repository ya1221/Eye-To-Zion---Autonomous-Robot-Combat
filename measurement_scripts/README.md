# measurement_scripts/

סקריפטי מדידה עבור כל פריט מסוג **[C]** (`דורש הרצה חדשה`) ב-[`../metrics_report.md`](../metrics_report.md). כל סקריפט עצמאי, כולל docstring/header עם PURPOSE / הרצה מדויקת / דרישות קדם / פורמט פלט צפוי (עם דוגמה מסומנת במפורש כאילוסטרטיבית — לא נתונים אמיתיים).

## התקנה כללית

```bash
# על הרובוט (RPi5) — לסקריפטי ROS2:
source /opt/ros/humble/setup.bash
source ~/AutonomousWarfare/ros2_ws/install/setup.bash

# לסקריפטי YOLO — סביבת Python עם ultralytics:
pip install ultralytics numpy
```

## מפתח: מטריקה ← סקריפט ← רשומה ביומן

| # | סקריפט | מודד | רשומה | חומרה נדרשת |
|---|---|---|---|---|
| 1 | [`bench_astar.py`](bench_astar.py) | זמן חישוב Hybrid A* (אחרי אופטימיזציה) + מס' צעדים + עמידה ב-100ms/500ms | 1 | כל מחשב (טהור Python, לא דורש ROS) |
| 2 | [`bench_rpi5_inference.py`](bench_rpi5_inference.py) | FPS וזמן אינפרנס YOLO26n-NCNN בפועל על RPi5 | 3 | RPi5 (או מחשב פיתוח להשוואה) |
| 3 | [`compare_yolo_models.py`](compare_yolo_models.py) | mAP@50/50-95 להשוואה בין מודלים/רזולוציות על אותו val set | 2, 3 | מחשב עם GPU (מומלץ) |
| 4 | [`train_yolo_ablation.sh`](train_yolo_ablation.sh) | mAP עם/בלי אוגמנטציה (הריפו מכיל ריצה אחת בלבד היום) | 2 | GPU (Kaggle/Colab/מקומי) |
| 5 | [`measure_zenoh_latency.py`](measure_zenoh_latency.py) | Latency, packet loss, jitter על טופיק `/teams/...` | 7 | 2 צמתים ברשת (רובוט + עמדת פיקוד) |
| 6 | [`measure_zenoh_recovery.sh`](measure_zenoh_recovery.sh) | זמן התאוששות אחרי ניתוק (מול baseline של 2s retry ב-Redis הישן) | 7 | 2 צמתים + הרשאות docker/iptables/tailscale |
| 7 | [`stability_soak_test.py`](stability_soak_test.py) | יציבות ארוכת-טווח / זמן עד קריסה, לפני/אחרי תיקון ה-race condition (`667a459`) | 8 | RPi5 (או מצב `--attach-live` מול תהליך אמיתי) |
| 8 | [`measure_driving_drift.py`](measure_driving_drift.py) | סחיפת נסיעה (ס"מ/מ') + הצעת `pwm_multiplier` לכל רובוט | 9 | **זירה פיזית + רובוט** ⚠️ מפעיל מנועים |
| 9 | [`measure_position_drift.py`](measure_position_drift.py) | פער סחיפה בין סימולציה (Gazebo) למציאות, על אותו מסלול מפוקד | 6 | **זירה + רובוט**, או Gazebo למצב sim ⚠️ מפעיל מנועים |
| 10 | [`measure_cumulative_error.py`](measure_cumulative_error.py) | שגיאה מצטברת עם/בלי תיקון ArUco, כפונקציה של מרחק | 15-20 | **זירה + מצלמת-על** — read-only, לא מפעיל מנועים |

> ⚠️ **סקריפטים 8-9 מפעילים מנועים על חומרה אמיתית.** כולם דורשים `--i-am-clear` מפורש, מבצעים ספירה לאחור, שולחים `Twist()` אפס ב-`finally`/SIGINT, ותומכים ב-`--dry-run`. **קרא את ה-docstring במלואו לפני הרצה ראשונה.**

## הרצה מהירה של כל מה שלא נוגע בחומרה

```bash
cd measurement_scripts
./bench_astar.py --repeats 20 --csv astar_results.csv
./stability_soak_test.py --no-lock --duration 30 --runs 3   # מדגים את הבאג (לפני התיקון)
./stability_soak_test.py --with-lock --duration 30 --runs 3 # מאמת שהתיקון מחזיק (ברירת מחדל)
```

## פלטים

כל סקריפט תומך ב-`--csv PATH` (ו/או `--json PATH`) לפלט הניתן לעיבוד; הפלט ל-stdout הוא תמיד טבלת סיכום קריאה. שרשרו כמה הרצות (`--robot-id`, `--layer`, `--mode`) לאותו קובץ CSV כדי לצבור טבלת השוואה אחת.

## סטטוס

כל 10 הסקריפטים נבדקו מבחינת syntax (`py_compile` / `bash -n`) ו-`--help`. חלקם נבדקו הרצה מלאה במצבי `--dry-run` / synthetic על ידי הסוכן שכתב אותם.

**שני סקריפטים הורצו בפועל ומעבר ל-syntax check** (על מחשב פיתוח x86_64 — **לא RPi5, לא חומרה/רשת אמיתית**), והתוצאות שלהם כבר שולבו ב-[`../metrics_report.md`](../metrics_report.md) עם ⚠️ הסתייגות ברורה שהן טעונות אימות מחדש על ה-RPi5 בפועל:

- **`bench_astar.py`** — הרצה אמיתית (לא סינתטית-בלבד) הראתה זמן תכנון קבוע בקירוב (~90–160ms ממוצע) לאורך צפיפויות מכשולים 200→3200 תאים, מה **שמוכיח בפועל שהאינדקס המרחבי עובד** (לפני האופטימיזציה: ~300,000ms סביב 1,600 תאים, לפי הקוד וההיסטוריה). 100ms **לא** הושג ב-p95, אך 500ms (הדדליין האמיתי) כן — בנוחות, ב-24/24 תרחישים.
- **`stability_soak_test.py --no-lock`** — שחזר את `RuntimeError: dictionary changed size during iteration` **בעקביות מלאה (2/2 הרצות)** תוך 0.002 שניות; `--with-lock` (ברירת המחדל, מצב הקוד הנוכחי) שרד 5+ שניות ו-~168K פעולות ללא תקלה. זו הדגמה נכונה של דפוס התחרות שהתועד ב-commit `667a459` — לא מדידת MTBF אמיתית על הרובוט.

**שאר 8 הסקריפטים לא הורצו בפועל מול חומרה, רשת, או זירה אמיתית** — התוצאות בקטע "EXPECTED OUTPUT FORMAT" בכל אחד מהם הן דוגמאות אילוסטרטיביות בלבד, לא מדידות.
