#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from enum import Enum
from visualization_msgs.msg import Marker
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
        rospy.Subscriber('/scan_filtered', LaserScan, self.lidar_callback)
        self.marker_pub = rospy.Publisher('debug_markers', Marker, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.marker_frame_id = "base_link"

        # 距離設定
        self.FORWARD_SAFE_DIST = 0.8
        self.FORWARD_AVOID_DIST = 0.5
        self.CRITICAL_STOP_DIST = 0.2
        self.SIDE_CLEAR_DIST = 0.65
        self.TARGET_SIDE_DIST = 0.40
        self.WALL_FOLLOW_KP_ANGULAR = 1.5

        # 速度設定
        self.LINEAR_SPEED_NORMAL = 0.12
        self.LINEAR_SPEED_SLOW = 0.05
        self.ANGULAR_SPEED_LOW = 0.5
        self.ANGULAR_SPEED_HIGH = 2.0

        self.LIDAR_DEAD_ZONE_RADIUS = 0.26

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
        dists = []
        scan_indices = []
        if angle_start_deg > angle_end_deg:
            for angle_deg in range(angle_start_deg, 360):
                relative_angle_rad = math.radians(angle_deg)
                idx = int((relative_angle_rad - angle_min) / angle_increment)
                scan_indices.append(idx)
            for angle_deg in range(0, angle_end_deg + 1):
                relative_angle_rad = math.radians(angle_deg)
                idx = int((relative_angle_rad - angle_min) / angle_increment)
                scan_indices.append(idx)
        else:
            for angle_deg in range(angle_start_deg, angle_end_deg + 1):
                relative_angle_rad = math.radians(angle_deg)
                idx = int((relative_angle_rad - angle_min) / angle_increment)
                scan_indices.append(idx)

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
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = alpha
        if text:
            marker.text = text
        self.marker_pub.publish(marker)

    def lidar_callback(self, scan):
        ranges = scan.ranges
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment

        self.front_center_dist = self._get_min_distance(ranges, angle_min, angle_increment, -10, 10)
        self.front_left_dist = self._get_min_distance(ranges, angle_min, angle_increment, 10, 40)
        self.front_right_dist = self._get_min_distance(ranges, angle_min, angle_increment, -40, -10)
        self.left_side_dist = self._get_min_distance(ranges, angle_min, angle_increment, 80, 100)
        self.right_side_dist = self._get_min_distance(ranges, angle_min, angle_increment, -100, -80)
        self.min_overall_dist = min(self.front_center_dist, self.front_left_dist, self.front_right_dist)

    def run(self):
        while not rospy.is_shutdown():
            cmd = Twist()

            # 状態遷移判定
            if self.min_overall_dist < self.CRITICAL_STOP_DIST:
                self.state = RobotState.CRITICAL_STOP
            elif self.front_center_dist < self.FORWARD_AVOID_DIST:
                self.state = RobotState.EVADE_OBSTACLE
            elif (self.left_side_dist < self.SIDE_CLEAR_DIST or self.right_side_dist < self.SIDE_CLEAR_DIST):
                self.state = RobotState.ADJUST_PATH
            else:
                self.state = RobotState.MOVE_FORWARD

            # 各状態の挙動
            if self.state == RobotState.MOVE_FORWARD:
                cmd.linear.x = self.LINEAR_SPEED_NORMAL
                cmd.angular.z = 0.0
                rospy.loginfo("Action: MOVE_FORWARD.")

            elif self.state == RobotState.ADJUST_PATH:
                cmd.linear.x = self.LINEAR_SPEED_NORMAL
                distance_error = self.right_side_dist - self.left_side_dist
                cmd.angular.z = self.WALL_FOLLOW_KP_ANGULAR * distance_error
                cmd.angular.z = max(-self.ANGULAR_SPEED_HIGH, min(cmd.angular.z, self.ANGULAR_SPEED_HIGH))
                rospy.loginfo(f"Action: ADJUST_PATH (Error:{distance_error:.2f} Ang_Z:{cmd.angular.z:.2f})")

                if self.left_side_dist < self.SIDE_CLEAR_DIST and self.right_side_dist > self.SIDE_CLEAR_DIST:
                    cmd.angular.z = -self.ANGULAR_SPEED_HIGH
                    cmd.linear.x = self.LINEAR_SPEED_SLOW
                    rospy.logwarn("ADJUST_PATH: Left too close, turning Right.")
                elif self.right_side_dist < self.SIDE_CLEAR_DIST and self.left_side_dist > self.SIDE_CLEAR_DIST:
                    cmd.angular.z = self.ANGULAR_SPEED_HIGH
                    cmd.linear.x = self.LINEAR_SPEED_SLOW
                    rospy.logwarn("ADJUST_PATH: Right too close, turning Left.")

            elif self.state == RobotState.EVADE_OBSTACLE:
                cmd.linear.x = self.LINEAR_SPEED_SLOW
                if self.front_left_dist > self.front_right_dist:
                    cmd.angular.z = self.ANGULAR_SPEED_HIGH
                    rospy.loginfo("EVADE_OBSTACLE: Turning Left.")
                else:
                    cmd.angular.z = -self.ANGULAR_SPEED_HIGH
                    rospy.loginfo("EVADE_OBSTACLE: Turning Right.")
                if self.front_left_dist < self.CRITICAL_STOP_DIST and self.front_right_dist < self.CRITICAL_STOP_DIST:
                    cmd.linear.x = 0.0
                    rospy.logerr("EVADE_OBSTACLE: Both sides blocked!")

            elif self.state == RobotState.CRITICAL_STOP:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                rospy.logfatal("CRITICAL_STOP: Robot halted.")

            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    node = None
    try:
        node = FreeSpaceNavigator()
        if node:
            node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("FreeSpaceNavigator: ROS Interrupt Exception caught.")
    except Exception as e:
        rospy.logerr(f"FreeSpaceNavigator: Unhandled exception in main - {e}")
    finally:
        if node and hasattr(node, 'cleanup') and callable(getattr(node, 'cleanup')):
            node.cleanup()
