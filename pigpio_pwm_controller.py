#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import pigpio
import time # timeモジュールをインポート

class PWMController:
    def __init__(self):
        rospy.init_node('pwm_controller')
        
        # pigpiodデーモンへの接続を試みる
        try:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise Exception("pigpiod not running or connection failed. Try 'sudo pigpiod'.")
            rospy.loginfo("Pigpio connection established.")
        except Exception as e:
            rospy.logerr(f"Failed to connect to pigpiod: {e}. Is pigpiod running? Try 'sudo pigpiod'.")
            rospy.signal_shutdown(f"Pigpio connection error: {e}")
            return # 初期化失敗でノードを終了

        # ★★★ GPIOピン番号 (実際のモータードライバ接続とステアリングに合わせる) ★★★
        # ここを正確にあなたのロボットの物理的なピン番号に修正してください！
        # 例: 納品時設定情報に基づいて、スロットルはGPIO 12、ステアリングはGPIO 13
        self.throttle_pin = 12  # 例: スピコン GPIO CH (モーター駆動)
        self.steer_pin = 13     # 例: ステアサーボ GPIO CH (ステアリング制御)

        # GPIOピンを出力モードに設定
        try:
            self.pi.set_mode(self.throttle_pin, pigpio.OUTPUT)
            self.pi.set_mode(self.steer_pin, pigpio.OUTPUT) 
            rospy.loginfo(f"GPIO pins Throttle:{self.throttle_pin}, Steer:{self.steer_pin} set to OUTPUT mode.")
        except Exception as e:
            rospy.logerr(f"Failed to set GPIO mode: {e}. Check pin numbers or if they are already in use.")
            self.pi.stop() 
            rospy.signal_shutdown(f"GPIO setup error: {e}")
            return

        # set_servo_pulsewidth を使うので、set_PWM_frequency/rangeは設定しない
        # pigpioはset_servo_pulsewidth内部で適切なPWM周波数(通常50Hz)を使用します。

        # ★★★ 初期状態でモーターとステアリングをニュートラルに設定 (1500μsを基準) ★★★
        # 「1450にしたら進み始めた」という情報があったが、まずは標準的な1500をニュートラルとする。
        # 後で必要なら1450や他の値を試す。
        self.neutral_pulse_throttle = 1500 # ★ 停止（ニュートラル）の基準パルス幅（マイクロ秒）★
        self.neutral_pulse_steer = 1500    # ★ ステアリングニュートラルパルス幅（マイクロ秒）★

        # 最大・最小パルス幅の定義 (通常のRC範囲)
        self.max_pulse = 2000 # 最大前進/右 (通常2000μs)
        self.min_pulse = 1000 # 最大後退/左 (通常1000μs)

        try:
            # set_servo_pulsewidth を使用して初期停止信号を送る
            self.pi.set_servo_pulsewidth(self.throttle_pin, self.neutral_pulse_throttle)
            rospy.loginfo(f"Throttle initialized to neutral pulsewidth: {self.neutral_pulse_throttle}us.")
            
            self.pi.set_servo_pulsewidth(self.steer_pin, self.neutral_pulse_steer)
            rospy.loginfo(f"Steering initialized to neutral pulsewidth: {self.neutral_pulse_steer}us.")

        except Exception as e:
            rospy.logerr(f"Failed to set initial pulsewidth: {e}.")
            self.pi.stop()
            rospy.signal_shutdown(f"Initial PWM error: {e}")
            return

        # /cmd_vel トピックの購読
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # FreeSpaceNavigatorの最大速度・最大角速度 (これらはFreeSpaceNavigator.pyの値と一致させる)
        self.max_linear_speed_from_freespace = 0.12 # FreeSpaceNavigatorのLINEAR_SPEED_NORMAL
        self.max_angular_speed_from_freespace = 2.0 # FreeSpaceNavigatorのANGULAR_SPEED_HIGH

        rospy.loginfo("PWMController node initialized and subscribing to /cmd_vel.")

    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # 前進/後退の方向を調整：ここでは符号反転は行わない (標準RCを想定)
        # もし前進コマンドでバックする場合は、ここで linear_vel = -linear_vel を再導入。

        # FreeSpaceNavigatorからの入力 (linear_vel) を -1.0 から 1.0 の範囲に正規化
        normalized_linear_vel = 0.0
        if self.max_linear_speed_from_freespace != 0:
            normalized_linear_vel = linear_vel / self.max_linear_speed_from_freespace
        
        # FreeSpaceNavigatorからの入力 (angular_vel) を -1.0 から 1.0 の範囲に正規化
        normalized_angular_vel = 0.0
        if self.max_angular_speed_from_freespace != 0:
            normalized_angular_vel = angular_vel / self.max_angular_speed_from_freespace
        
        # ★★★ スロットルパルス幅を計算 (1500μs中心) ★★★
        # normalized_linear_vel = 1.0 (最大前進) -> 2000μs
        # normalized_linear_vel = -1.0 (最大後退) -> 1000μs
        pulse_throttle = self.convert_to_pulse_throttle(normalized_linear_vel)
        
        # ★★★ ステアリングパルス幅を計算 (1500μs中心) ★★★
        # normalized_angular_vel = 1.0 (最大左) -> 2000μs (例)
        # normalized_angular_vel = -1.0 (最大右) -> 1000μs (例)
        # ステアリングの左右とジョイスティックの左右が逆の場合、normalized_angular_velの符号を反転させる (例: -normalized_angular_vel)
        pulse_steer = self.convert_to_pulse_steer(normalized_angular_vel) 
        
        try:
            self.pi.set_servo_pulsewidth(self.throttle_pin, pulse_throttle)
            self.pi.set_servo_pulsewidth(self.steer_pin, pulse_steer) 
            
            # デバッグログ
            rospy.loginfo(f"CmdVel Rcvd: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f} | "
                          f"Throttle Pulse: {pulse_throttle}us (Norm:{normalized_linear_vel:.2f}) | "
                          f"Steer Pulse: {pulse_steer}us (Norm:{normalized_angular_vel:.2f})")
        except Exception as e:
            rospy.logerr(f"Failed to set pulsewidth in callback: {e}. Pin: T={self.throttle_pin}, S={self.steer_pin}")

    # ★★★ スロットル用のパルス幅変換関数 (1000-2000μs) ★★★
    def convert_to_pulse_throttle(self, normalized_speed):
        # normalized_speed は -1.0 から 1.0 の範囲を想定
        clamped_speed = max(min(normalized_speed, 1.0), -1.0)
        # 1500μsを中心に±500μsの範囲で変換
        pulse = int(clamped_speed * 500 + self.neutral_pulse_throttle) 
        return max(self.min_pulse, min(pulse, self.max_pulse)) 

    # ★★★ ステアリング用のパルス幅変換関数 (1000-2000μs) ★★★
    def convert_to_pulse_steer(self, normalized_angular_vel):
        # normalized_angular_vel は -1.0 から 1.0 の範囲を想定
        clamped_norm_angular = max(min(normalized_angular_vel, 1.0), -1.0)
        # 1500μsを中心に±500μsの範囲で変換
        pulse = int(clamped_norm_angular * 500 + self.neutral_pulse_steer) 
        return max(self.min_pulse, min(pulse, self.max_pulse))

    def cleanup(self):
        rospy.loginfo("PWMController: Cleaning up GPIO and stopping pigpio.")
        try:
            # モーターとステアリングをニュートラルに
            self.pi.set_servo_pulsewidth(self.throttle_pin, self.neutral_pulse_throttle) 
            self.pi.set_servo_pulsewidth(self.steer_pin, self.neutral_pulse_steer) 
            time.sleep(0.5) # ESCが信号を認識する時間を与える
            
            # PWM信号を完全にオフにする (0は無効信号)
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
