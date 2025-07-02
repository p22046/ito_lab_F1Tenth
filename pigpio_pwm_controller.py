#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import pigpio
import time

class PWMController:
    def __init__(self):
        rospy.init_node('pwm_controller')
        
        try:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise Exception("pigpiod not running or connection failed. Try 'sudo pigpiod'.")
            rospy.loginfo("Pigpio connection established.")
        except Exception as e:
            rospy.logerr(f"Failed to connect to pigpiod: {e}. Is pigpiod running? Try 'sudo pigpiod'.")
            rospy.signal_shutdown(f"Pigpio connection error: {e}")
            return

        self.throttle_pin = 12
        self.steer_pin = 13

        try:
            self.pi.set_mode(self.throttle_pin, pigpio.OUTPUT)
            self.pi.set_mode(self.steer_pin, pigpio.OUTPUT) 
            rospy.loginfo(f"GPIO pins Throttle:{self.throttle_pin}, Steer:{self.steer_pin} set to OUTPUT mode.")
        except Exception as e:
            rospy.logerr(f"Failed to set GPIO mode: {e}. Check pin numbers or if they are already in use.")
            self.pi.stop() 
            rospy.signal_shutdown(f"GPIO setup error: {e}")
            return

        # ★★★ ここからPWM周波数の設定に関する部分を削除またはコメントアウトします ★★★
        # set_PWM_frequency と set_PWM_range の呼び出しを削除またはコメントアウト
        # self.pwm_frequency = 70 # Hz
        # self.pwm_range_value = 40000 
        # try:
        #     self.pi.set_PWM_frequency(self.throttle_pin, self.pwm_frequency)
        #     self.pi.set_PWM_range(self.throttle_pin, self.pwm_range_value)
        #     self.pi.set_PWM_frequency(self.steer_pin, self.pwm_frequency)
        #     self.pi.set_PWM_range(self.steer_pin, self.pwm_range_value)
        #     rospy.loginfo(f"PWM frequency set to {self.pwm_frequency}Hz for both pins, range {self.pwm_range_value}.")
        # except Exception as e:
        #     rospy.logerr(f"Failed to set PWM frequency or range: {e}.")
        #     self.pi.stop()
        #     rospy.signal_shutdown(f"PWM frequency setup error: {e}")
        #     return
        
        # ★★★ パルス幅 (マイクロ秒) の定数定義はそのまま使用 ★★★
        # set_servo_pulsewidth を使うので Dutycycle の計算は不要
        self.neutral_pulse_throttle_us = 1475 # 停止パルス (us)
        self.neutral_pulse_steer_us = 1500    # ステアリングニュートラルパルス (us)

        self.max_pulse_forward_us = 1450 # ★ 最大前進時のパルス幅 (us) (ここを調整して速度を落とす) ★
        self.min_pulse_backward_us = 1000 # 後退の最小パルス幅 (us)
        
        self.max_pulse_steer_us = 2000 # ステアリングの最大パルス幅 (us)
        self.min_pulse_steer_us = 1000 # ステアリングの最小パルス幅 (us)

        try:
            # ★★★ set_servo_pulsewidth を使用して初期停止信号を送る ★★★
            self.pi.set_servo_pulsewidth(self.throttle_pin, self.neutral_pulse_throttle_us)
            rospy.loginfo(f"Throttle initialized to neutral pulsewidth: {self.neutral_pulse_throttle_us}us.")
            
            self.pi.set_servo_pulsewidth(self.steer_pin, self.neutral_pulse_steer_us)
            rospy.loginfo(f"Steering initialized to neutral pulsewidth: {self.neutral_pulse_steer_us}us.")

        except Exception as e:
            rospy.logerr(f"Failed to set initial pulsewidth: {e}.")
            self.pi.stop()
            rospy.signal_shutdown(f"Initial PWM error: {e}")
            return

        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        self.max_linear_speed_from_freespace = 0.12 # FreeSpaceNavigatorのLINEAR_SPEED_NORMAL
        self.max_angular_speed_from_freespace = 2.0 # FreeSpaceNavigatorのANGULAR_SPEED_HIGH

        rospy.loginfo("PWMController node initialized and subscribing to /cmd_vel.")

    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        normalized_linear_vel = 0.0
        if self.max_linear_speed_from_freespace != 0:
            normalized_linear_vel = linear_vel / self.max_linear_speed_from_freespace
        
        normalized_angular_vel = 0.0
        if self.max_angular_speed_from_freespace != 0:
            normalized_angular_vel = angular_vel / self.max_angular_speed_from_freespace
        
        # パルス幅 (us) を計算
        pulse_throttle = self.convert_to_pulse_throttle(normalized_linear_vel) # 関数名も変更
        pulse_steer = self.convert_to_pulse_steer(normalized_angular_vel)       # 関数名も変更
        
        try:
            # ★★★ set_servo_pulsewidth を使用してパルス幅を出力 ★★★
            self.pi.set_servo_pulsewidth(self.throttle_pin, pulse_throttle)
            self.pi.set_servo_pulsewidth(self.steer_pin, pulse_steer) 
            
            # デバッグログ ( Dutycycle値は不要、パルス幅usのみ表示 )
            rospy.loginfo(f"CmdVel Rcvd: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f} | "
                            f"Throttle Pulse: {pulse_throttle}us (Norm:{normalized_linear_vel:.2f}) | "
                            f"Steer Pulse: {pulse_steer}us (Norm:{normalized_angular_vel:.2f})")
        except Exception as e:
            rospy.logerr(f"Failed to set pulsewidth in callback: {e}. Pin: T={self.throttle_pin}, S={self.steer_pin}")

    # ★★★ スロットル用のパルス幅変換関数 (1475usで停止、1300-1450usで前進) ★★★
    def convert_to_pulse_throttle(self, normalized_speed):
        clamped_speed = max(0.0, min(normalized_speed, 1.0)) 

        # 停止パルス: self.neutral_pulse_throttle_us = 1475
        # 最大前進パルス: self.max_pulse_forward_us = 1420 (調整値)
        
        # normalized_speedが 0.0 のとき 1475us
        # normalized_speedが 1.0 のとき 1450us
        pulse = int( (1 - clamped_speed) * self.neutral_pulse_throttle_us + \
                     clamped_speed * self.max_pulse_forward_us )
        
        return max(self.max_pulse_forward_us, min(pulse, self.neutral_pulse_throttle_us))

    # ★★★ ステアリング用のパルス幅変換関数 (1000us-2000us範囲) ★★★
    def convert_to_pulse_steer(self, normalized_angular_vel):
        clamped_norm_angular = max(min(normalized_angular_vel, 1.0), -1.0)
        
        # -1.0 (最大左) -> min_pulse_steer_us
        # 0.0 (ニュートラル) -> neutral_pulse_steer_us
        # 1.0 (最大右) -> max_pulse_steer_us
        
        pulse = int(clamped_norm_angular * (self.max_pulse_steer_us - self.neutral_pulse_steer_us) / 1.0 + self.neutral_pulse_steer_us)
        
        return max(self.min_pulse_steer_us, min(pulse, self.max_pulse_steer_us))

    def cleanup(self):
        rospy.loginfo("PWMController: Cleaning up GPIO and stopping pigpio.")
        try:
            # PWM信号をニュートラル、その後完全にオフにする
            self.pi.set_servo_pulsewidth(self.throttle_pin, self.neutral_pulse_throttle_us) 
            self.pi.set_servo_pulsewidth(self.steer_pin, self.neutral_pulse_steer_us) 
            time.sleep(0.5) 
            
            self.pi.set_servo_pulsewidth(self.throttle_pin, 0) # 0 は無効信号 (多くのESCは信号途絶を検出)
            self.pi.set_servo_pulsewidth(self.steer_pin, 0) 
            time.sleep(0.1) 

            self.pi.stop() 
            rospy.loginfo("PWMController: GPIO cleanup successful and pigpio stopped.")
        except Exception as e:
            rospy.logerr(f"Error during PWMController cleanup: {e}. GPIOs might not be fully released.")

if __name__ == '__main__':
    controller = None 
    try:
        controller = PWMController()
        if controller and controller.pi and controller.pi.connected:
            rospy.spin() 
    except rospy.ROSInterruptException:
        rospy.loginfo("PWMController: ROS Interrupt Exception caught.")
    except Exception as e:
        rospy.logerr(f"PWMController: Unhandled exception in main - {e}")
    finally:
        if controller: 
            controller.cleanup()
