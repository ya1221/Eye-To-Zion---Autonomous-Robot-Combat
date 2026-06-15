import math
import queue # תור לניהול הודעות Zenoh
import zenoh # ספריית P2P

# שים לב ששינינו מ-redis_manager ל-zenoh_manager
from tactical_brain import zenoh_manager
from tactical_brain import A_planner

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import py_trees


# from A_alg import MAX_COMBAT_DISTANCE
MAX_COMBAT_DISTANCE = 5.0 # ערך זמני עד שנשלב את שאר הקבצים

#######################
# father action class
class Nav2BaseAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node=None):
        super().__init__(name)
        self.ros_node = ros_node
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        if self.ros_node:
            # אנחנו עכשיו מדברים ישירות עם הבקר המקומי (Controller) של Nav2
            self._action_client = ActionClient(self.ros_node, FollowPath, 'follow_path')
        self._is_driving = False

    def get_tactical_path(self):
        """
        חייב להחזיר רשימה של נקודות: [(x, y, yaw), (x, y, yaw), ...]
        """
        raise NotImplementedError("Child classes must implement get_tactical_path!")

    def update(self):
        if self.ros_node is None: return py_trees.common.Status.SUCCESS
        if self._is_driving: return py_trees.common.Status.RUNNING
        
        waypoints = self.get_tactical_path()
        if not waypoints: 
            return py_trees.common.Status.FAILURE
            
        # בניית הודעת מסלול (Path) תקנית של ROS2
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        
        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            # המרת הזווית (Yaw) לקווטרניון ש-ROS2 דורש
            pose.pose.orientation.z = float(math.sin(yaw / 2.0))
            pose.pose.orientation.w = float(math.cos(yaw / 2.0))
            path_msg.poses.append(pose)

        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        goal_msg.controller_id = 'FollowPath' # השם הסטנדרטי ב-Nav2
        
        self.ros_node.get_logger().info(f"[{self.name}] Sending Tactical Path ({len(waypoints)} points) to Nav2")
        self._action_client.send_goal_async(goal_msg)
        self._is_driving = True
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and self._is_driving:
            if self.ros_node:
                self.ros_node.get_logger().info(f"[{self.name}] Interrupted! Canceling FollowPath.")
            self._is_driving = False

#######################
# attack branch
class IsEnemyClose(py_trees.behaviour.Behaviour):
    def __init__(self, name = "is_enemy_close"):
        super(IsEnemyClose, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="dist_to_closest_enemy", access=py_trees.common.Access.READ)

    def update(self):
        dist= self.blackboard.dist_to_closest_enemy
        if dist is not None and dist <= MAX_COMBAT_DISTANCE:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
    
class HasLineOfSightToEnemy(py_trees.behaviour.Behaviour):
    def __init__(self, name = "has_line_of_sight_to_enemy"):
        super(HasLineOfSightToEnemy, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="has_line_of_sight_to_closest_enemy", access=py_trees.common.Access.READ)

    def update(self):
        has_line_of_sight_to_enemy = self.blackboard.has_line_of_sight_to_closest_enemy
        if has_line_of_sight_to_enemy is not None and has_line_of_sight_to_enemy:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
    
class AttackAction(py_trees.behaviour.Behaviour):
    def __init__(self, name = "attack_action", ros_node=None):
        super(AttackAction, self).__init__(name)
        self.ros_node = ros_node
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="robot_command", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="current_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_y", access=py_trees.common.Access.READ)

    def update(self):
        self.blackboard.robot_command = "ATTACK"
        self.ros_node.get_logger().info(f"[{self.name}] Engaging enemy at ({self.blackboard.current_x:.2f}, {self.blackboard.current_y:.2f})")
        # shooting logic
        return py_trees.common.Status.SUCCESS
    
#######################
# survival branch
class IsLowHealthOrOutnumbered(py_trees.behaviour.Behaviour):
    def __init__(self, name = "is_low_health_or_outnumbered"):
        super(IsLowHealthOrOutnumbered, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="health", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="num_enemies", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="num_teammates", access=py_trees.common.Access.READ)

    def update(self):
        health = self.blackboard.health
        num_enemies = self.blackboard.num_enemies
        num_teammates = self.blackboard.num_teammates

        if (health is not None and health < 0.5) or (num_enemies is not None and num_teammates is not None and num_enemies > num_teammates + 1):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class AreTeammatesComing(py_trees.behaviour.Behaviour):
    def __init__(self, name="are_teammates_coming"):
        super(AreTeammatesComing, self).__init__(name)
    def update(self):
        return py_trees.common.Status.FAILURE # נניח שכרגע אף אחד לא בא

class HideAndWaitAction(Nav2BaseAction):
    def __init__(self, name="hide_and_wait_action", ros_node=None):
        super().__init__(name, ros_node)
        self.blackboard.register_key(key="robot_command", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="current_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_yaw", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="hide_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="hide_y", access=py_trees.common.Access.READ)
        
    def get_tactical_path(self):
        cx, cy = self.blackboard.current_x, self.blackboard.current_y
        cyaw = self.blackboard.current_yaw
        hx, hy = self.blackboard.hide_x, self.blackboard.hide_y
        
        if hx is not None and hy is not None and self.ros_node is not None:
            self.blackboard.robot_command = "HIDING (A* PATH)"
            
            start_pos = (cx, cy, cyaw)
            goal_pos = (hx, hy, 0.0) # כיוון לא משנה במחסה
            
            self.ros_node.get_logger().info(f"[{self.name}] Calculating A* evasion route...")
            
            result = A_planner.calc_hybrid_a_star(
                start=start_pos,
                goal=goal_pos,
                obstacle_set=self.ros_node.static_obstacles,
                xy_resolution=A_planner.XY_RESOLUTION,
                yaw_resolution=A_planner.YAW_RESOLUTION,
                danger_dict=self.ros_node.danger_dict,
                teammates_aura_set=self.ros_node.teammates_aura_set
            )
            
            if result is None:
                self.ros_node.get_logger().warn(f"[{self.name}] No safe evasion path found!")
                return None
                
            path_obj, _ = result
            x_path, y_path, theta_path, _ = path_obj
            
            tactical_waypoints = [(x_path[i], y_path[i], theta_path[i]) for i in range(len(x_path))]
            return tactical_waypoints
            
        return None
        
class RunFromEnemyAction(Nav2BaseAction):
    def __init__(self, name="run_from_enemy_action", ros_node=None):
        super().__init__(name, ros_node)
        self.blackboard.register_key(key="robot_command", access=py_trees.common.Access.WRITE)
        
    def get_tactical_path(self):
        self.blackboard.robot_command = "RUNNING_FROM_ENEMY"
        # מחזיר מסלול עם נקודה אחת כדי לא לקרוס (עד שנחבר את A*)
        return [(0.0, 0.0, 0.0)]
    
#######################
# help teammate branch
class IsTeammateInDanger(py_trees.behaviour.Behaviour):
    def __init__(self, name = "is_teammate_in_danger"):
        super(IsTeammateInDanger, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key = "teammate_requested_help", access = py_trees.common.Access.READ)
    
    def update(self):
        teammate_requested_help = self.blackboard.teammate_requested_help
        if teammate_requested_help is not None and teammate_requested_help:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IsDistanceToTeammateAcceptable(py_trees.behaviour.Behaviour):
    def __init__(self, name = "is_distance_to_teammate_acceptable"):
        super(IsDistanceToTeammateAcceptable, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="dist_to_help_teammate", access=py_trees.common.Access.READ)

    def update(self):
        dist_to_help_teammate = self.blackboard.dist_to_help_teammate
        if dist_to_help_teammate is not None and dist_to_help_teammate < 5.0:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class DriveToTeammateAction(Nav2BaseAction):
    def __init__(self, name="drive_to_teammate_action", ros_node=None):
        super().__init__(name, ros_node)
        self.blackboard.register_key(key="current_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_yaw", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="teammate_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="teammate_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="robot_command", access=py_trees.common.Access.WRITE)

    def get_tactical_path(self):
        cx, cy = self.blackboard.current_x, self.blackboard.current_y
        cyaw = self.blackboard.current_yaw
        tx, ty = self.blackboard.teammate_x, self.blackboard.teammate_y
        
        if tx is not None and ty is not None and self.ros_node is not None:
            self.blackboard.robot_command = "DRIVE_TO_TEAMMATE (A* PATH)"
            
            start_pos = (cx, cy, cyaw)
            goal_pos = (tx, ty, 0.0) 
            
            self.ros_node.get_logger().info(f"[{self.name}] Calculating A* rescue route...")
            
            result = A_planner.calc_hybrid_a_star(
                start=start_pos,
                goal=goal_pos,
                obstacle_set=self.ros_node.static_obstacles,
                xy_resolution=A_planner.XY_RESOLUTION,
                yaw_resolution=A_planner.YAW_RESOLUTION,
                danger_dict=self.ros_node.danger_dict,
                teammates_aura_set=self.ros_node.teammates_aura_set
            )
            
            if result is None:
                self.ros_node.get_logger().warn(f"[{self.name}] No path to teammate found!")
                return None
                
            path_obj, _ = result
            x_path, y_path, theta_path, _ = path_obj
            
            tactical_waypoints = [(x_path[i], y_path[i], theta_path[i]) for i in range(len(x_path))]
            return tactical_waypoints
            
        return None
#########################
# patrol branch
class MapsToGoalAction(Nav2BaseAction): 
    def __init__(self, name="maps_to_goal_action", ros_node=None):
        super().__init__(name, ros_node)
        self.blackboard.register_key(key="current_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_yaw", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="goal_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="goal_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="robot_command", access=py_trees.common.Access.WRITE)

    def get_tactical_path(self):
        cx, cy = self.blackboard.current_x, self.blackboard.current_y
        cyaw = self.blackboard.current_yaw
        gx, gy = self.blackboard.goal_x, self.blackboard.goal_y
        
        if gx is not None and gy is not None and self.ros_node is not None:
            self.blackboard.robot_command = "NAVIGATING (A* PATH)"
            
            start_pos = (cx, cy, cyaw)
            goal_pos = (gx, gy, 0.0) # בינתיים נניח שיעד הסיור לא דורש זווית ספציפית
            
            self.ros_node.get_logger().info(f"[{self.name}] Running Hybrid A* planner...")
            
            # קריאה לאלגוריתם שלכם
            result = A_planner.calc_hybrid_a_star(
                start=start_pos,
                goal=goal_pos,
                obstacle_set=self.ros_node.static_obstacles,
                xy_resolution=A_planner.XY_RESOLUTION,
                yaw_resolution=A_planner.YAW_RESOLUTION,
                danger_dict=self.ros_node.danger_dict,
                teammates_aura_set=self.ros_node.teammates_aura_set
            )
            
            if result is None:
                self.ros_node.get_logger().warn(f"[{self.name}] A* Planner failed to find a path!")
                return None
                
            path_obj, cost = result
            x_path, y_path, theta_path, directions = path_obj
            
            self.ros_node.get_logger().info(f"[{self.name}] A* found path with {len(x_path)} steps. Cost: {cost:.2f}")
            
            # אריזת הנתונים לפורמט [(x, y, yaw), ...] עבור Nav2BaseAction
            tactical_waypoints = []
            for i in range(len(x_path)):
                tactical_waypoints.append((x_path[i], y_path[i], theta_path[i]))
                
            return tactical_waypoints
            
        return None
    
# ==========================================
# פונקציית יצירת העץ המקורית שלך
# ==========================================
def create_tree(ros_node=None):
    survival_branch = py_trees.composites.Sequence(name="survival_branch", memory=False)
    tactical_reaction_selector = py_trees.composites.Selector(name="tactical_reaction_selector", memory=False)
    
    wait_for_teammates_branch = py_trees.composites.Sequence(name="wait_for_teammates_branch", memory=False)
    wait_for_teammates_branch.add_children([
        AreTeammatesComing(name="teammates coming?"),
        HideAndWaitAction(name="hide and wait action", ros_node=ros_node)
    ])
    
    tactical_reaction_selector.add_children([
        wait_for_teammates_branch,               
        RunFromEnemyAction(name="run from enemy action", ros_node=ros_node),
    ])
    
    survival_branch.add_children([
        IsLowHealthOrOutnumbered(name="in danger?"), 
        tactical_reaction_selector                   
    ])

    attack_branch = py_trees.composites.Sequence(name="attack_branch", memory = False)
    attack_branch.add_children(
        [IsEnemyClose(name = "is enemy close"), 
         HasLineOfSightToEnemy(name = "has line of sight to enemy"),
         AttackAction(name = "attack action", ros_node=ros_node)])

    help_teammate_branch = py_trees.composites.Sequence(name="help_teammate_branch", memory = False)
    help_teammate_branch.add_children(
        [IsTeammateInDanger(name = "is teammate in danger"), 
         IsDistanceToTeammateAcceptable(name = "is distance to teammate acceptable"), 
         DriveToTeammateAction(name = "drive to teammate action", ros_node=ros_node)])

    patrol_branch = py_trees.composites.Sequence(name="patrol_branch", memory = False)
    patrol_branch.add_children(
        [MapsToGoalAction(name = "maps to goal action", ros_node=ros_node)])

    root = py_trees.composites.Selector(name="root", memory = False)
    root.add_children([survival_branch, attack_branch, help_teammate_branch, patrol_branch])

    return py_trees.trees.BehaviourTree(root)


# ==========================================
# קוד ה-ROS2 Node (המעטפת שמריצה ובודקת את העץ)
# # ==========================================
# class TacticalBrainNode(Node):
#     def __init__(self):
#         super().__init__('tactical_brain_node')
#         self.get_logger().info("Tactical Brain is waking up and planting the tree...")
        
#         self.redis_client = redis.Redis(host='redis_server', port=6379, db=0) # שנה IP במידת הצורך
#         self.pubsub = self.redis_client.pubsub()
#         self.pubsub.subscribe('/detected_enemies', '/team/positions', '/aruco/poses') 
    
#         self.pose_sub = self.create_subscription(
#             PoseWithCovarianceStamped,
#             '/amcl_pose',  
#             self.pose_callback,
#             10
#         )

#         # יצירת ה-Subscriber כדי לקרוא את מה שה-redis_manager משדר
#         self.aruco_pose_sub = self.create_subscription(
#             PoseStamped,
#             '/sensor_fusion_node/aruco_global_pose',
#             self.aruco_callback,
#             10
#         )
#          # משתנים לשמירת האמת האבסולוטית מהארוקו
#         self.aruco_x = 0.0
#         self.aruco_y = 0.0
#         self.aruco_yaw = 0.0

#         # משתנים לשמירת הסטייה המחושבת
#         self.drift_x = 0.0
#         self.drift_y = 0.0
#         self.drift_yaw = 0.0

#         # פבלישר לערוץ חטיפת המיקום של מערכת הניווט
#         self.initial_pose_pub = self.create_publisher(
#             PoseWithCovarianceStamped,
#             '/initialpose',
#             10
#         )

#         # 2. אתחול משתני מצב מקומיים (מהקוד שלך)
#         self.static_obstacles = set() # (כאן תיכנס מפת הקירות מהלידאר בעתיד)
#         self.enemies_list = []
#         self.teammates_dict = {}
#         self.danger_dict = {}
#         self.teammates_aura_set = set()

#         # 1. רישום כל המשתנים שהעץ שלך צריך ב-Blackboard
#         self.blackboard = py_trees.blackboard.Client(name="global")
#         keys = [
#             "current_x", "current_y", "current_yaw", "hide_x", "hide_y", "goal_x", "goal_y", "teammate_x", "teammate_y",
#             "dist_to_closest_enemy", "has_line_of_sight_to_closest_enemy", 
#             "health", "num_enemies", "num_teammates", 
#             "teammate_requested_help", "dist_to_help_teammate", "robot_command"
#         ]
#         for key in keys:
#             self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
            
#         # מצב התחלתי שקט
#         self.reset_blackboard_to_safe_state()

#         # 2. הקמת העץ
#         self.tree = create_tree(ros_node=self)
#         self.tree.setup(timeout=15)

#         # 3. טיימרים
#         # טיימר שמפעיל את העץ כל חצי שנייה (2Hz)
#         self.tree_timer = self.create_timer(0.5, self.sense_and_think)

#         # טיימר שמשנה את התרחיש כל 5 שניות
#         #self.scenario_timer = self.create_timer(5.0, self.mock_scenario_changer)
#         #self.scenario_step = 0

#     def reset_blackboard_to_safe_state(self):
#         self.blackboard.dist_to_closest_enemy = 100.0
#         self.blackboard.has_line_of_sight_to_closest_enemy = False
#         self.blackboard.health = 1.0
#         self.blackboard.num_enemies = 0
#         self.blackboard.num_teammates = 1
#         self.blackboard.teammate_requested_help = False
#         self.blackboard.dist_to_help_teammate = 100.0
#         self.blackboard.robot_command = "WAIT"

#         self.blackboard.current_x = 2.0
#         self.blackboard.current_y = 2.0
#         self.blackboard.current_yaw = 0.0
#         self.blackboard.hide_x = 2.25
#         self.blackboard.hide_y = 4.75
#         self.blackboard.goal_x = 3.5
#         self.blackboard.goal_y = 4.0
#         self.blackboard.teammate_x = 1.5
#         self.blackboard.teammate_y = 1.0
        
#     def sense_and_think(self):
#         """
#         זוהי לולאת הליבה של הרובוט: Sense -> Think -> Act
#         """
#         # שלב א': Sense (קריאה מהרדיס באמצעות הפונקציה שלך)
#         self.danger_dict, self.teammates_aura_set, self.enemies_list, self.teammates_dict = redis_manager.get_latest_world_state(
#             self.pubsub, self.enemies_list, self.teammates_dict, self.static_obstacles
#         )
        
#         # שלב ב': Translation (תרגום המילונים למשתנים פשוטים שעץ ההתנהגות מבין)
#         self.update_blackboard_from_redis_state()
        
#         # שלב ג': Think (הפעלת העץ לקבלת החלטה)
#         self.tree.tick()
#         self.get_logger().info(f"Tree Output Command ---> {self.blackboard.robot_command}")
    

#     def update_blackboard_from_redis_state(self):
#         # 1. עדכון נתוני אויבים
#         self.blackboard.num_enemies = len(self.enemies_list)
        
#         if self.enemies_list:
#             current_pos = (self.blackboard.current_x, self.blackboard.current_y)
            
#             # 1. פיצול האויבים לשתי קבוצות: גלויים ומוסתרים
#             visible_enemies = []
#             hidden_enemies = []
            
#             for enemy in self.enemies_list:
#                 enemy_pos = (enemy['x'], enemy['y'])
#                 if redis_manager.line_of_sight_clear(current_pos, enemy_pos, self.static_obstacles):
#                     visible_enemies.append(enemy)
#                 else:
#                     hidden_enemies.append(enemy)
            
#             # 2. בחירת האיום העיקרי לפי עדיפות טקטית
#             if visible_enemies:
#                 # יש אויבים גלויים! ניקח את הקרוב מביניהם
#                 closest_enemy = min(visible_enemies, key=lambda e: distance(current_pos, (e['x'], e['y'])))
#                 self.blackboard.has_line_of_sight_to_closest_enemy = True
#             else:
#                 # כולם מוסתרים. ניקח את המוסתר הקרוב ביותר (למשל כדי להתכונן להסתערות שלו)
#                 closest_enemy = min(hidden_enemies, key=lambda e: distance(current_pos, (e['x'], e['y'])))
#                 self.blackboard.has_line_of_sight_to_closest_enemy = False
            
#             # 3. עדכון המרחק בלוח
#             self.blackboard.dist_to_closest_enemy = distance(current_pos, (closest_enemy['x'], closest_enemy['y']))
#         else:
#             # אין אויבים באזור
#             self.blackboard.dist_to_closest_enemy = 100.0
#             self.blackboard.has_line_of_sight_to_closest_enemy = False

#         # 2. עדכון נתוני חברי צוות
#         self.blackboard.num_teammates = len(self.teammates_dict)
#         if self.teammates_dict:
#             # ניקח כרגע את החבר הראשון במילון לשם הדוגמה
#             first_teammate_id = list(self.teammates_dict.keys())[0]
#             teammate_data = self.teammates_dict[first_teammate_id]
            
#             self.blackboard.teammate_x = teammate_data.get('x')
#             self.blackboard.teammate_y = teammate_data.get('y')
            
#             # אם הוא שידר דגל מצוקה ברדיס
#             self.blackboard.teammate_requested_help = teammate_data.get('needs_help', False)

    
#     def aruco_callback(self, msg):
#         # קליטת קואורדינטות האמת
#         self.aruco_x = msg.pose.position.x
#         self.aruco_y = msg.pose.position.y
#         self.aruco_yaw = self.get_yaw_from_quaternion(msg.pose.orientation)
        
#         # חישוב הסטייה מול ה-Blackboard (AMCL)
#         self.drift_x = self.aruco_x - self.blackboard.current_x
#         self.drift_y = self.aruco_y - self.blackboard.current_y
#         yaw_diff = self.aruco_yaw - self.blackboard.current_yaw
#         self.drift_yaw = math.atan2(math.sin(yaw_diff), math.cos(yaw_diff))
        
#         # חישוב המרחק הכולל (היפוטנוזה של משולש X,Y)
#         total_drift_meters = math.hypot(self.drift_x, self.drift_y)
        
#         # אם הסטייה עולה על 5 ס"מ (0.05 מטר), אנחנו מבצעים איפוס קשיח!
#         if total_drift_meters > 0.05:
#             self.get_logger().warn(f"HIGH ODOM DRIFT: {total_drift_meters*100:.1f}cm! Resetting AMCL to ArUco ground truth.")
            
#             # יצירת הודעת האיפוס
#             reset_msg = PoseWithCovarianceStamped()
#             reset_msg.header.stamp = self.get_clock().now().to_msg()
#             reset_msg.header.frame_id = 'map'
            
#             # הזנת קואורדינטות הארוקו לתוך ה-AMCL
#             reset_msg.pose.pose.position.x = self.aruco_x
#             reset_msg.pose.pose.position.y = self.aruco_y
#             reset_msg.pose.pose.orientation = msg.pose.orientation
            
#             # (אופציונלי אך מומלץ): איפוס מטריצת השגיאות (Covariance) לזיהוי מדויק
#             reset_msg.pose.covariance[0] = 0.05 # רדיוס ביטחון סביר
#             reset_msg.pose.covariance[7] = 0.05
#             reset_msg.pose.covariance[35] = 0.05
            
#             # שיגור ההוראה למערכת הניווט!
#             self.initial_pose_pub.publish(reset_msg)
            
#             # איפוס המשתנים המקומיים כדי שלא נשגר איפוס כפול בפריים הבא
#             self.drift_x = 0.0
#             self.drift_y = 0.0

#     def pose_callback(self, msg):
#         # שימוש בפונקציית העזר כדי שהקוד יהיה נקי
#         self.blackboard.current_x = msg.pose.pose.position.x
#         self.blackboard.current_y = msg.pose.pose.position.y
#         self.blackboard.current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

#     def get_yaw_from_quaternion(self, q):
#         siny_cosp = 2 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
#         return math.atan2(siny_cosp, cosy_cosp)

#     # def mock_scenario_changer(self):
#     #     self.scenario_step += 1
#     #     print("\n" + "="*50)
        
#     #     if self.scenario_step == 1:
#     #         self.get_logger().info("SCENARIO 1: All clear. Expecting to Patrol.")
#     #         self.reset_blackboard_to_safe_state()
            
#     #     elif self.scenario_step == 2:
#     #         self.get_logger().info("SCENARIO 2: Enemy 2m away in line of sight! Expecting Combat.")
#     #         self.reset_blackboard_to_safe_state()
#     #         self.blackboard.dist_to_closest_enemy = 2.0
#     #         self.blackboard.has_line_of_sight_to_closest_enemy = True
            
#     #     elif self.scenario_step == 3:
#     #         self.get_logger().info("SCENARIO 3: Teammate in danger 3m away! Expecting to Help.")
#     #         self.reset_blackboard_to_safe_state()
#     #         self.blackboard.teammate_requested_help = True
#     #         self.blackboard.dist_to_help_teammate = 3.0
            
#     #     elif self.scenario_step == 4:
#     #         self.get_logger().info("SCENARIO 4: Ambushed! 3 enemies, low health. Expecting to Run/Hide.")
#     #         self.reset_blackboard_to_safe_state()
#     #         self.blackboard.health = 0.3
#     #         self.blackboard.num_enemies = 3
#     #         self.blackboard.num_teammates = 1
            
#     #     else:
#     #         self.get_logger().info("Restarting scenarios...")
#     #         self.scenario_step = 0

# ==========================================
# קוד ה-ROS2 Node (המעטפת שמריצה ובודקת את העץ)
# ==========================================
class TacticalBrainNode(Node):
    def __init__(self):
        super().__init__('tactical_brain_node')
        self.get_logger().info("Tactical Brain is waking up and planting the tree...")
        
        # ---------------------------------------------------------
        # שינוי 1: מחיקת הרדיס והחלפה בהגדרות Zenoh + Queue
        # ---------------------------------------------------------
        self.msg_queue = queue.Queue()
        conf = zenoh.Config()
        conf.insert_json5("connect/endpoints", '["tcp/100.107.5.41:7447"]')
        self.zenoh_session = zenoh.open(conf) # חיבור אוטומטי לרשת עם קונפיגורציה        

        # פונקציית הקולבק של Zenoh שדוחפת הודעות לתור שלנו
        def zenoh_listener(sample):
            self.msg_queue.put({
                "channel": str(sample.key_expr),
                "data": bytes(sample.payload).decode('utf-8')
            })
            
        # הרשמה לערוצים - הוספנו קידומת (Namespace) לקבוצה הכחולה!
        TEAM_PREFIX = "team_blue"
        self.zenoh_session.declare_subscriber(f'{TEAM_PREFIX}/detected_enemies', zenoh_listener)
        self.zenoh_session.declare_subscriber(f'{TEAM_PREFIX}/team_positions', zenoh_listener)
        self.zenoh_session.declare_subscriber(f'{TEAM_PREFIX}/fleet_positions', zenoh_listener)
        # ---------------------------------------------------------

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',  
            self.pose_callback,
            10
        )

        # יצירת ה-Subscriber כדי לקרוא את מה שהמנג'ר משדר
        self.aruco_pose_sub = self.create_subscription(
            PoseStamped,
            '/sensor_fusion_node/aruco_global_pose',
            self.aruco_callback,
            10
        )
        
        # פבלישר לערוץ של הארוקו (כדי שה-zenoh_manager יוכל לפרסם אליו)
        self.aruco_pub = self.create_publisher(
            PoseStamped, 
            '/sensor_fusion_node/aruco_global_pose', 
            10
        )

        # משתנים לשמירת האמת האבסולוטית מהארוקו
        self.aruco_x = 0.0
        self.aruco_y = 0.0
        self.aruco_yaw = 0.0

        # משתנים לשמירת הסטייה המחושבת
        self.drift_x = 0.0
        self.drift_y = 0.0
        self.drift_yaw = 0.0

        # פבלישר לערוץ חטיפת המיקום של מערכת הניווט (איפוס סטייה)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # 2. אתחול משתני מצב מקומיים 
        self.static_obstacles = set() 
        self.enemies_list = []
        self.teammates_dict = {}
        self.danger_dict = {}
        self.teammates_aura_set = set()

        # 1. רישום כל המשתנים שהעץ צריך ב-Blackboard
        self.blackboard = py_trees.blackboard.Client(name="global")
        keys = [
            "current_x", "current_y", "current_yaw", "hide_x", "hide_y", "goal_x", "goal_y", "teammate_x", "teammate_y",
            "dist_to_closest_enemy", "has_line_of_sight_to_closest_enemy", 
            "health", "num_enemies", "num_teammates", 
            "teammate_requested_help", "dist_to_help_teammate", "robot_command"
        ]
        for key in keys:
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
            
        # מצב התחלתי שקט
        self.reset_blackboard_to_safe_state()

        # 2. הקמת העץ
        self.tree = create_tree(ros_node=self)
        self.tree.setup(timeout=15)

        # 3. טיימרים
        self.tree_timer = self.create_timer(0.5, self.sense_and_think)


    def reset_blackboard_to_safe_state(self):
        self.blackboard.dist_to_closest_enemy = 100.0
        self.blackboard.has_line_of_sight_to_closest_enemy = False
        self.blackboard.health = 1.0
        self.blackboard.num_enemies = 0
        self.blackboard.num_teammates = 1
        self.blackboard.teammate_requested_help = False
        self.blackboard.dist_to_help_teammate = 100.0
        self.blackboard.robot_command = "WAIT"

        self.blackboard.current_x = 2.0
        self.blackboard.current_y = 2.0
        self.blackboard.current_yaw = 0.0
        self.blackboard.hide_x = 2.25
        self.blackboard.hide_y = 4.75
        self.blackboard.goal_x = 3.5
        self.blackboard.goal_y = 4.0
        self.blackboard.teammate_x = 1.5
        self.blackboard.teammate_y = 1.0
        
    def sense_and_think(self):
        """
        זוהי לולאת הליבה של הרובוט: Sense -> Think -> Act
        """
        # שלב א': Sense (קריאה מהתור של Zenoh במקום הרדיס)
        # שים לב שהעברנו את self.msg_queue וגם את ה-publisher של הארוקו
        self.danger_dict, self.teammates_aura_set, self.enemies_list, self.teammates_dict = zenoh_manager.get_latest_world_state(
            self.msg_queue, 
            self.enemies_list, 
            self.teammates_dict, 
            self.static_obstacles,
            ros_node=self, 
            aruco_pub=self.aruco_pub
        )
        
        # שלב ב': Translation 
        self.update_blackboard_from_zenoh_state()
        
        # שלב ג': Think
        self.tree.tick()
        self.get_logger().info(f"Tree Output Command ---> {self.blackboard.robot_command}")
    

    def update_blackboard_from_zenoh_state(self):
        # 1. עדכון נתוני אויבים
        self.blackboard.num_enemies = len(self.enemies_list)
        
        if self.enemies_list:
            current_pos = (self.blackboard.current_x, self.blackboard.current_y)
            
            visible_enemies = []
            hidden_enemies = []
            
            for enemy in self.enemies_list:
                enemy_pos = (enemy['x'], enemy['y'])
                # שים לב לשינוי הקריאה מ-redis_manager ל-zenoh_manager
                if zenoh_manager.line_of_sight_clear(current_pos, enemy_pos, self.static_obstacles):
                    visible_enemies.append(enemy)
                else:
                    hidden_enemies.append(enemy)
            
            if visible_enemies:
                closest_enemy = min(visible_enemies, key=lambda e: distance(current_pos, (e['x'], e['y'])))
                self.blackboard.has_line_of_sight_to_closest_enemy = True
            else:
                closest_enemy = min(hidden_enemies, key=lambda e: distance(current_pos, (e['x'], e['y'])))
                self.blackboard.has_line_of_sight_to_closest_enemy = False
            
            self.blackboard.dist_to_closest_enemy = distance(current_pos, (closest_enemy['x'], closest_enemy['y']))
        else:
            self.blackboard.dist_to_closest_enemy = 100.0
            self.blackboard.has_line_of_sight_to_closest_enemy = False

        # 2. עדכון נתוני חברי צוות
        self.blackboard.num_teammates = len(self.teammates_dict)
        if self.teammates_dict:
            first_teammate_id = list(self.teammates_dict.keys())[0]
            teammate_data = self.teammates_dict[first_teammate_id]
            
            self.blackboard.teammate_x = teammate_data.get('x')
            self.blackboard.teammate_y = teammate_data.get('y')
            self.blackboard.teammate_requested_help = teammate_data.get('needs_help', False)

    
    def aruco_callback(self, msg):
        self.aruco_x = msg.pose.position.x
        self.aruco_y = msg.pose.position.y
        self.aruco_yaw = self.get_yaw_from_quaternion(msg.pose.orientation)
        
        self.drift_x = self.aruco_x - self.blackboard.current_x
        self.drift_y = self.aruco_y - self.blackboard.current_y
        yaw_diff = self.aruco_yaw - self.blackboard.current_yaw
        self.drift_yaw = math.atan2(math.sin(yaw_diff), math.cos(yaw_diff))
        
        total_drift_meters = math.hypot(self.drift_x, self.drift_y)
        
        if total_drift_meters > 0.05:
            self.get_logger().warn(f"HIGH ODOM DRIFT: {total_drift_meters*100:.1f}cm! Resetting AMCL to ArUco ground truth.")
            
            reset_msg = PoseWithCovarianceStamped()
            reset_msg.header.stamp = self.get_clock().now().to_msg()
            reset_msg.header.frame_id = 'map'
            
            reset_msg.pose.pose.position.x = self.aruco_x
            reset_msg.pose.pose.position.y = self.aruco_y
            reset_msg.pose.pose.orientation = msg.pose.orientation
            
            reset_msg.pose.covariance[0] = 0.05 
            reset_msg.pose.covariance[7] = 0.05
            reset_msg.pose.covariance[35] = 0.05
            
            self.initial_pose_pub.publish(reset_msg)
            
            self.drift_x = 0.0
            self.drift_y = 0.0

    def pose_callback(self, msg):
        self.blackboard.current_x = msg.pose.pose.position.x
        self.blackboard.current_y = msg.pose.pose.position.y
        self.blackboard.current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])
    
def main(args=None):
    rclpy.init(args=args)
    node = TacticalBrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()