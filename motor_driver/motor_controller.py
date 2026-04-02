import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import smbus
import time

# PCA9685 Registers
PCA9685_ADDRESS = 0x40
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06


class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')

        # Subscriber
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_joy',
            self.cmd_vel_callback,
            10
        )

        # I2C
        self.bus = smbus.SMBus(1)
        self.address = PCA9685_ADDRESS

        self.init_pwm()

        # ESC CONFIG
        self.min_us = 1000
        self.max_us = 2000
        self.neutral_us = 1500

        # CONTROL LIMITS
        self.max_linear = 1.0
        self.max_angular = 1.0

        # DEAD BAND
        self.deadband = 0.05

        # SMOOTHING
        self.alpha = 0.2  # lower = smoother
        self.prev_left = 0.0
        self.prev_right = 0.0

        # SAFETY TIMEOUT
        self.last_cmd_time = time.time()
        self.timeout = 0.5  # seconds
        self.timer = self.create_timer(0.1, self.safety_check)

        # REVERSE PROTECTION
        self.prev_left_sign = 0
        self.prev_right_sign = 0

        self.get_logger().info("🚀 Motor Driver Node (Advanced) Started")

    # ---------------- PWM INIT ----------------
    def init_pwm(self):
        self.bus.write_byte_data(self.address, MODE1, 0x00)
        time.sleep(0.005)

        freq = 50
        prescale = int(25000000.0 / (4096 * freq) - 1)

        old_mode = self.bus.read_byte_data(self.address, MODE1)
        new_mode = (old_mode & 0x7F) | 0x10
        self.bus.write_byte_data(self.address, MODE1, new_mode)
        self.bus.write_byte_data(self.address, PRESCALE, prescale)
        self.bus.write_byte_data(self.address, MODE1, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, MODE1, old_mode | 0x80)

    # ---------------- UTILS ----------------
    def us_to_12bit(self, pulse_us):
        pulse_length = 20000.0 / 4096
        return int(pulse_us / pulse_length)

    def set_pwm(self, channel, value):
        reg = LED0_ON_L + 4 * channel

        self.bus.write_byte_data(self.address, reg, 0 & 0xFF)
        self.bus.write_byte_data(self.address, reg + 1, 0 >> 8)
        self.bus.write_byte_data(self.address, reg + 2, value & 0xFF)
        self.bus.write_byte_data(self.address, reg + 3, value >> 8)

    # ---------------- SAFETY ----------------
    def safety_check(self):
        if time.time() - self.last_cmd_time > self.timeout:
            self.stop_motors()

    def stop_motors(self):
        neutral_val = self.us_to_12bit(self.neutral_us)
        self.set_pwm(0, neutral_val)
        self.set_pwm(1, neutral_val)
        self.get_logger().warn("⚠️ Safety Stop Activated")

    # ---------------- MAIN CALLBACK ----------------
    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()

        linear = msg.linear.x
        angular = msg.angular.z

        # DEAD BAND
        if abs(linear) < self.deadband:
            linear = 0.0
        if abs(angular) < self.deadband:
            angular = 0.0

        # NORMALIZE
        linear = max(min(linear, self.max_linear), -self.max_linear)
        angular = max(min(angular, self.max_angular), -self.max_angular)

        # DIFFERENTIAL DRIVE
        left = linear - angular
        right = linear + angular

        left = max(min(left, 1.0), -1.0)
        right = max(min(right, 1.0), -1.0)

        # SMOOTHING (Low-pass filter)
        left = self.alpha * left + (1 - self.alpha) * self.prev_left
        right = self.alpha * right + (1 - self.alpha) * self.prev_right

        self.prev_left = left
        self.prev_right = right

        # REVERSE PROTECTION
        left_sign = 1 if left > 0 else -1 if left < 0 else 0
        right_sign = 1 if right > 0 else -1 if right < 0 else 0

        if left_sign != self.prev_left_sign:
            left = 0.0
            time.sleep(0.05)

        if right_sign != self.prev_right_sign:
            right = 0.0
            time.sleep(0.05)

        self.prev_left_sign = left_sign
        self.prev_right_sign = right_sign

        # CONVERT TO PWM
        left_us = self.neutral_us + left * (self.max_us - self.neutral_us)
        right_us = self.neutral_us + right * (self.max_us - self.neutral_us)

        left_val = self.us_to_12bit(left_us)
        right_val = self.us_to_12bit(right_us)

        # SEND TO PWM MODULE
        self.set_pwm(0, left_val)
        self.set_pwm(1, right_val)

        self.get_logger().info(
            f"L:{left:.2f} ({left_val}) | R:{right:.2f} ({right_val})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()