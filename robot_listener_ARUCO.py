import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os

class RobotTacticalListener(Node):
    def __init__(self, my_id, team_topic):
        super().__init__(f'robot_{my_id}_listener')
        self.my_id = my_id
        
        # הרשמה רק לטופיק של הקבוצה שלי
        self.subscription = self.create_subscription(
            String,
            team_topic,
            self.listener_callback,
            10
        )
        self.get_logger().info(f'Robot {self.my_id} listening to {team_topic}')

    def listener_callback(self, msg):
        try:
            team_positions = json.loads(msg.data)
            
            # מחלץ את המיקום שלי
            my_pos = next((p for p in team_positions if p['id'] == self.my_id), None)
            
            # מחלץ את המיקום של שאר חברי הצוות (להימנעות מהתנגשויות או מבנה טקטי)
            allies = [p for p in team_positions if p['id'] != self.my_id]
            
            if my_pos:
                # כאן אתה מעביר את הנתונים ללוגיקת התנועה (PID, Path Planning וכו')
                self.get_logger().info(f'Me [X:{my_pos["x"]}, Y:{my_pos["y"]}, A:{my_pos["angle"]}] | Allies visible: {len(allies)}')
            else:
                self.get_logger().warn('Warning: Not detected in this frame!')
                
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse incoming JSON data.')

def main(args=None):
    rclpy.init(args=args)
    
    MY_ID = int(os.environ.get('ROBOT_ID', 4))
    MY_TEAM = os.environ.get('ROBOT_TEAM', 'alpha') # 'alpha' or 'beta'
    
    MY_TEAM_TOPIC = f'teams/alliance_{MY_TEAM}/positions'
    
    tactical_node = RobotTacticalListener(my_id=MY_ID, team_topic=MY_TEAM_TOPIC)
    
    try:
        rclpy.spin(tactical_node)
    except KeyboardInterrupt:
        pass
    finally:
        tactical_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
