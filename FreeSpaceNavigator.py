#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from enum import Enum
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf 

class RobotState(Enum):
    MOVE_FORWARD = 1        
    ADJUST_PATH = 2         
    EVADE_OBSTACLE = 3      
    CRITICAL_STOP = 4       

class FreeSpaceNavigator:
    def __init__(self):
        rospy.init_node('free_space_navigator')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # フィルタリングされたLidarデータを購読
        rospy.Subscriber('/scan_filtered', LaserScan, self.lidar_callback)
        self.marker_pub = rospy.Publisher('debug_markers', Marker, queue_size=1)
        
        self.tf_listener = tf.TransformListener()
        self.marker_frame_id = "base_link" 

        # ★ 障害物検知と回避に関する閾値 (LiDARの性能やロボットのサイズに合わせて調整してください)
        self.FORWARD_SAFE_DIST = 0.8    
        self.FORWARD_AVOID_DIST = 0.5   
        self.CRITICAL_STOP_DIST = 0.2   
        self.SIDE_CLEAR_DIST = 0.65     # 0.45 から 0.65 へ増加 (さらに早めに側面回避を開始)
        self.CORNER_DETECT_ANGLE_RANGE = 30 

        # ★ 左右の壁追従に関する新しい定数 
        self.TARGET_SIDE_DIST = 0.40    # 左右の壁から維持したい目標距離 (廊下の中央など、調整が必要)
        self.WALL_FOLLOW_KP_ANGULAR = 1.5 # 壁追従の操舵ゲイン (P制御のゲイン、調整が必要)

        # ★ 速度設定 (ロボットの運動性能に合わせて調整してください)
        self.LINEAR_SPEED_NORMAL = 0.12   # 0.16 から 0.12 へ減少 (通常速度も少し遅くして余裕を持たせる)
        self.LINEAR_SPEED_SLOW = 0.05   # 0.10 から 0.05 へ大幅に減少 (回避中の速度をさらにさらに落とす)
        self.ANGULAR_SPEED_LOW = 0.5    
        self.ANGULAR_SPEED_HIGH = 2.0   # 1.0 から 2.0 へ大幅に増加 (回避中の旋回をさらに急にする)

        # LIDAR_DEAD_ZONE_RADIUS がここで確実に初期化されていることを確認
        self.LIDAR_DEAD_ZONE_RADIUS = 0.26 # 半径15cm (0.15m) 以内を無視。調整が必要ならここを修正

        self.state = RobotState.MOVE_FORWARD 

        self.front_center_dist = float('inf') 
        self.front_left_dist = float('inf')    
        self.front_right_dist = float('inf')   
        self.left_side_dist = float('inf')      
        self.right_side_dist = float('inf')    
        self.min_overall_dist = float('inf') 

        self.rate = rospy.Rate(10) 

        rospy.loginfo("FreeSpaceNavigator Node Initialized.")

    def _get_min_distance(self, ranges, angle_min, angle_increment, angle_start_deg, angle_end_deg):
        """指定された角度範囲の最小距離を安全に取得するヘルパー関数"""
