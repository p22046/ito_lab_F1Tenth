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
        dists = []
        
        # 360度をまたぐ場合
        if angle_start_deg > angle_end_deg: 
            scan_indices = []
            for angle_deg in range(angle_start_deg, 360):
                relative_angle_rad = math.radians(angle_deg) 
                idx = int((relative_angle_rad - angle_min) / angle_increment)

                if idx < 0:
                    idx += len(ranges)
                elif idx >= len(ranges):
                    idx -= len(ranges)
                scan_indices.append(idx)

            for angle_deg in range(0, angle_end_deg + 1):
                relative_angle_rad = math.radians(angle_deg)
                idx = int((relative_angle_rad - angle_min) / angle_increment)
                if idx < 0:
                    idx += len(ranges)
                elif idx >= len(ranges):
                    idx -= len(ranges)
                scan_indices.append(idx)
        else: # 360度をまたがない場合
            scan_indices = []
            for angle_deg in range(angle_start_deg, angle_end_deg + 1):
                relative_angle_rad = math.radians(angle_deg)
                idx = int((relative_angle_rad - angle_min) / angle_increment)
                if idx < 0:
                    idx += len(ranges)
                elif idx >= len(ranges):
                    idx -= len(ranges)
                scan_indices.append(idx)
        
        # 収集したインデックスから有効な距離を抽出
        for idx in scan_indices:
            if 0 <= idx < len(ranges): 
                r = ranges[idx]
                if not math.isinf(r) and not math.isnan(r) and r > self.LIDAR_DEAD_ZONE_RADIUS:
                    dists.append(r)
        
        return min(dists) if dists else float('inf')


    def _publish_marker(self, marker_id, x, y, z, r=0.0, g=0.0, b=0.0, alpha=1.0, scale=0.1, text="", marker_type=Marker.SPHERE):
        marker = Marker()
        marker.header.frame_id = self.marker_frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "lidar_debug_markers" 
        marker.id = marker_id
        marker.type = marker_type 
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0 

        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = alpha

        if text and marker_type == Marker.TEXT_VIEW_FACING:
            marker.text = text
            marker.scale.z = scale * 2 

        self.marker_pub.publish(marker)

    def lidar_callback(self, msg): 
        # Lidarの0度がロボットの後方の場合の角度調整を反映
        # ロボットの「前方」はLidarの180度付近
        self.front_center_dist = self._get_min_distance(msg.ranges, msg.angle_min, msg.angle_increment, 170, 190) 
        
        # ロボットの「左前方」 (Lidarの180度から左に30度～45度)
        self.front_left_dist = self._get_min_distance(msg.ranges, msg.angle_min, msg.angle_increment, 135, 170)

        # ロボットの「右前方」 (Lidarの180度から右に30度～45度)
        self.front_right_dist = self._get_min_distance(msg.ranges, msg.angle_min, msg.angle_increment, 190, 225)

        # ロボットの「左側面」 (Lidarの-90～-45度方向,つまり270～315度方向)
        self.left_side_dist = self._get_min_distance(msg.ranges, msg.angle_min, msg.angle_increment, -90, -45)  

        # ロボットの「右側面」 (Lidarの45～90度方向)
        self.right_side_dist = self._get_min_distance(msg.ranges, msg.angle_min, msg.angle_increment, 45, 90)   
        
        all_valid_ranges = []
        for r in msg.ranges:
            if not math.isinf(r) and not math.isnan(r) and r > self.LIDAR_DEAD_ZONE_RADIUS:
                all_valid_ranges.append(r)
        self.min_overall_dist = min(all_valid_ranges) if all_valid_ranges else float('inf')

        # デバッグログの出力
        rospy.loginfo(
            f"State: {self.state.name} | "
            f"Front: FC={self.front_center_dist:.2f}m (L:{self.front_left_dist:.2f}m, R:{self.front_right_dist:.2f}m) | "
            f"Side: L={self.left_side_dist:.2f}m, R:{self.right_side_dist:.2f}m | "
            f"Overall Min: {self.min_overall_dist:.2f}m"
        )
        rospy.loginfo(
            f"Thresholds: Safe={self.FORWARD_SAFE_DIST:.2f}, Avoid={self.FORWARD_AVOID_DIST:.2f}, Critical={self.CRITICAL_STOP_DIST:.2f}, SideClear={self.SIDE_CLEAR_DIST:.2f} | DeadZone={self.LIDAR_DEAD_ZONE_RADIUS:.2f}"
        )

        # マーカーのパブリッシュ
        try:
            # base_link から laser_frame へのTF変換を取得
            self.tf_listener.waitForTransform("base_link", "laser_frame", rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform('base_link', 'laser_frame', rospy.Time(0))
            
            laser_x_offset = trans[0]
            laser_y_offset = trans[1]
            laser_z_offset = trans[2]

            # 各方向のマーカーをパブリッシュ (ロボットのbase_link座標系上で直感的に見える位置に表示)
            # 前方中央マーカー (ロボットの真前、base_linkのX軸方向)
            self._publish_marker(1, laser_x_offset + self.front_center_dist * math.cos(math.radians(0)), laser_y_offset + self.front_center_dist * math.sin(math.radians(0)), laser_z_offset, r=1.0, g=0.0, b=0.0, scale=0.05) 
            self._publish_marker(11, laser_x_offset + self.front_center_dist * math.cos(math.radians(0)), laser_y_offset + self.front_center_dist * math.sin(math.radians(0)), laser_z_offset + 0.1, r=1.0, g=0.0, b=0.0, scale=0.03, text=f"FC:{self.front_center_dist:.2f}", marker_type=Marker.TEXT_VIEW_FACING)

            # 左前方マーカー (ロボットの左前方30度、base_linkのX+Y方向)
            self._publish_marker(2, laser_x_offset + self.front_left_dist * math.cos(math.radians(30)), laser_y_offset + self.front_left_dist * math.sin(math.radians(30)), laser_z_offset, r=0.0, g=1.0, b=0.0, scale=0.05) 
            self._publish_marker(12, laser_x_offset + self.front_left_dist * math.cos(math.radians(30)), laser_y_offset + self.front_left_dist * math.sin(math.radians(30)), laser_z_offset + 0.1, r=0.0, g=1.0, b=0.0, scale=0.03, text=f"FL:{self.front_left_dist:.2f}", marker_type=Marker.TEXT_VIEW_FACING)

            # 右前方マーカー (ロボットの右前方-30度、base_linkのX-Y方向)
            self._publish_marker(3, laser_x_offset + self.front_right_dist * math.cos(math.radians(-30)), laser_y_offset + self.front_right_dist * math.sin(math.radians(-30)), laser_z_offset, r=0.0, g=0.0, b=1.0, scale=0.05) 
            self._publish_marker(13, laser_x_offset + self.front_right_dist * math.cos(math.radians(-30)), laser_y_offset + self.front_right_dist * math.sin(math.radians(-30)), laser_z_offset + 0.1, r=0.0, g=0.0, b=1.0, scale=0.03, text=f"FR:{self.front_right_dist:.2f}", marker_type=Marker.TEXT_VIEW_FACING)

            # 左側面マーカー (ロボットの真左90度、base_linkの+Y方向)
            self._publish_marker(4, laser_x_offset + self.left_side_dist * math.cos(math.radians(90)), laser_y_offset + self.left_side_dist * math.sin(math.radians(90)), laser_z_offset, r=1.0, g=1.0, b=0.0, scale=0.05) 
            self._publish_marker(14, laser_x_offset + self.left_side_dist * math.cos(math.radians(90)), laser_y_offset + self.left_side_dist * math.sin(math.radians(90)), laser_z_offset + 0.1, r=1.0, g=1.0, b=0.0, scale=0.03, text=f"SL:{self.left_side_dist:.2f}", marker_type=Marker.TEXT_VIEW_FACING)

            # 右側面マーカー (ロボットの真右-90度、base_linkの-Y方向)
            self._publish_marker(5, laser_x_offset + self.right_side_dist * math.cos(math.radians(-90)), laser_y_offset + self.right_side_dist * math.sin(math.radians(-90)), laser_z_offset, r=1.0, g=0.5, b=0.0, scale=0.05) 
            self._publish_marker(15, laser_x_offset + self.right_side_dist * math.cos(math.radians(-90)), laser_y_offset + self.right_side_dist * math.sin(math.radians(-90)), laser_z_offset + 0.1, r=1.0, g=0.5, b=0.0, scale=0.03, text=f"SR:{self.right_side_dist:.2f}", marker_type=Marker.TEXT_VIEW_FACING)

            # 全周最小距離マーカー (Lidarの実際の角度で表示)
            if all_valid_ranges and self.min_overall_dist != float('inf'):
                try:
                    min_overall_idx = list(msg.ranges).index(self.min_overall_dist) 
                    min_overall_angle_rad = msg.angle_min + min_overall_idx * msg.angle_increment
                    min_overall_x = laser_x_offset + self.min_overall_dist * math.cos(min_overall_angle_rad)
                    min_overall_y = laser_y_offset + self.min_overall_dist * math.sin(min_overall_angle_rad)
                    self._publish_marker(6, min_overall_x, min_overall_y, laser_z_offset, r=1.0, g=0.0, b=1.0, scale=0.07) 
                    self._publish_marker(16, min_overall_x, min_overall_y, laser_z_offset + 0.1, r=1.0, g=0.0, b=1.0, scale=0.03, text=f"Min:{self.min_overall_dist:.2f}", marker_type=Marker.TEXT_VIEW_FACING)
                except ValueError: 
                    rospy.logwarn_throttle(1, "min_overall_dist not found in original ranges for marker placement.")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, f"TF Transform not available for publishing markers: {e}. Is base_link and laser_frame defined?")
            pass 
        
    def run(self):
        cmd = Twist()
        rate = self.rate

        while not rospy.is_shutdown():
            # --- 状態遷移ロジック（優先度順） ---

            # 1. 最優先: 真前が緊急停止距離以下の場合
            if self.front_center_dist < self.CRITICAL_STOP_DIST:
                self.state = RobotState.CRITICAL_STOP
                rospy.logfatal(f"CRITICAL STOP: Front center {self.front_center_dist:.2f}m. Halting robot.")
            
            # 2. 前方に障害物があり、回避が必要な場合 (Evade_Obstacle または Adjust_Pathへ)
            elif self.front_center_dist < self.FORWARD_AVOID_DIST or \
                 self.front_left_dist < self.FORWARD_AVOID_DIST or \
                 self.front_right_dist < self.FORWARD_AVOID_DIST:
                
                if self.front_center_dist < self.FORWARD_AVOID_DIST:
                    self.state = RobotState.EVADE_OBSTACLE
                    rospy.logwarn(f"EVADE_OBSTACLE: Front blocked ({self.front_center_dist:.2f}m).")
                elif self.state == RobotState.MOVE_FORWARD or self.state == RobotState.ADJUST_PATH:
                    self.state = RobotState.ADJUST_PATH
                    rospy.loginfo(f"ADJUST_PATH: Obstacle nearby ({self.front_left_dist:.2f}/{self.front_right_dist:.2f}m).")
                
            # 3. 回避/調整中から回復: 進路が十分クリアになったらMOVE_FORWARDへ
            elif self.state in [RobotState.ADJUST_PATH, RobotState.EVADE_OBSTACLE]:
                if self.front_center_dist > self.FORWARD_SAFE_DIST * 1.2 and \
                   self.front_left_dist > self.FORWARD_SAFE_DIST and \
                   self.front_right_dist > self.FORWARD_SAFE_DIST:
                    self.state = RobotState.MOVE_FORWARD
                    rospy.loginfo("MOVE_FORWARD: Path clear, returning to straight.")
            
            # 4. デフォルト: 障害物がなければ直進
            else:
                self.state = RobotState.MOVE_FORWARD


            # --- 各状態での行動決定 ---

            if self.state == RobotState.MOVE_FORWARD:
                cmd.linear.x = self.LINEAR_SPEED_NORMAL
                cmd.angular.z = 0.0
                
            elif self.state == RobotState.ADJUST_PATH:
                cmd.linear.x = self.LINEAR_SPEED_NORMAL # 速度を落とさず微調整

                # 左右の壁からの距離の誤差を計算
                # ターゲット距離との差を使って、中央に寄せるように操舵
                distance_error = self.right_side_dist - self.left_side_dist

                # 比例制御で角速度を計算
                cmd.angular.z = self.WALL_FOLLOW_KP_ANGULAR * distance_error
                
                # 角速度を最大値でクランプ (設定したANGULAR_SPEED_HIGHを超えないように)
                cmd.angular.z = max(-self.ANGULAR_SPEED_HIGH, min(cmd.angular.z, self.ANGULAR_SPEED_HIGH))

                rospy.loginfo(f"Action: ADJUST_PATH (Wall Following Error:{distance_error:.2f} Ang_Z:{cmd.angular.z:.2f}).")
                
                # 壁への接近に対する優先度の高い回避ロジック（そのまま残す）
                if self.left_side_dist < self.SIDE_CLEAR_DIST and self.right_side_dist > self.SIDE_CLEAR_DIST: # 左が狭く、右はクリア
                    cmd.angular.z = -self.ANGULAR_SPEED_HIGH # 右へ強めに旋回して左の壁から離れる
                    cmd.linear.x = self.LINEAR_SPEED_SLOW # 速度も落とす
                    rospy.logwarn(f"Action: ADJUST_PATH (Left side too close, steering Right harder. L:{self.left_side_dist:.2f}).")
                elif self.right_side_dist < self.SIDE_CLEAR_DIST and self.left_side_dist > self.SIDE_CLEAR_DIST: # 右が狭く、左はクリア
                    cmd.angular.z = self.ANGULAR_SPEED_HIGH # 左へ強めに旋回して右の壁から離れる
                    cmd.linear.x = self.LINEAR_SPEED_SLOW # 速度も落とす
                    rospy.logwarn(f"Action: ADJUST_PATH (Right side too close, steering Left harder. R:{self.right_side_dist:.2f}).")

            elif self.state == RobotState.EVADE_OBSTACLE:
                cmd.linear.x = self.LINEAR_SPEED_SLOW # 減速して回避
                if self.front_left_dist > self.front_right_dist:
                    cmd.angular.z = self.ANGULAR_SPEED_HIGH # 左へ大きく旋回
                    rospy.loginfo("Action: EVADE_OBSTACLE (Turning Left).")
                else:
                    cmd.angular.z = -self.ANGULAR_SPEED_HIGH # 右へ大きく旋回
                    rospy.loginfo("Action: EVADE_OBSTACLE (Turning Right).")
                
                if self.front_left_dist < self.CRITICAL_STOP_DIST and self.front_right_dist < self.CRITICAL_STOP_DIST:
                    cmd.linear.x = 0.0 
                    rospy.logerr("Action: EVADE_OBSTACLE (Both sides blocked, near stop).")

            elif self.state == RobotState.CRITICAL_STOP:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                rospy.logfatal("Action: CRITICAL_STOP. Robot Halted.")

            self.cmd_pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    node = None # 初期化前にNoneで定義 (安全のため)
    try:
        node = FreeSpaceNavigator() 
        if node: # ノードが正常に初期化された場合のみrun()を呼び出す
            node.run() # 
    except rospy.ROSInterruptException:
        rospy.loginfo("FreeSpaceNavigator: ROS Interrupt Exception caught.") 
    except Exception as e: 
        rospy.logerr(f"FreeSpaceNavigator: Unhandled exception in main - {e}")
    finally: 
        # FreeSpaceNavigatorクラスにはcleanupメソッドがないため、この部分は通常実行されないが、形式として残す
        if node and hasattr(node, 'cleanup') and callable(getattr(node, 'cleanup')):
            node.cleanup() 
