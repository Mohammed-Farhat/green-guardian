import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import smbus2
import time
import math

# MPU-6500 register map
REG_PWR_MGMT_1  = 0x6B
REG_ACCEL_XOUT  = 0x3B
REG_GYRO_XOUT   = 0x43
REG_TEMP_OUT    = 0x41
REG_ACCEL_CFG   = 0x1C
REG_GYRO_CFG    = 0x1B

# Scale factors for default ranges
# Accel: ±2g  → 16384 LSB/g
# Gyro:  ±250°/s → 131 LSB/°/s
ACCEL_SCALE = 16384.0
GYRO_SCALE  = 131.0
GRAVITY     = 9.80665  # m/s²
DEG_TO_RAD  = math.pi / 180.0


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # Parameters
        self.declare_parameter('i2c_bus',   1)
        self.declare_parameter('i2c_addr',  0x68)
        self.declare_parameter('publish_rate', 50.0)   # Hz
        self.declare_parameter('frame_id',  'imu_link')
        self.declare_parameter('calibration_samples', 200)  # 4s at 50Hz

        bus_num  = self.get_parameter('i2c_bus').value
        self.addr = self.get_parameter('i2c_addr').value
        rate_hz  = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        calib_n  = self.get_parameter('calibration_samples').value

        # I2C
        self.bus = smbus2.SMBus(bus_num)
        self._init_chip()

        # Gyro bias calibration — robot must be stationary during startup.
        # Uncalibrated MPU-6500 gyro bias is 0.01–0.05 rad/s; integrated by the
        # EKF this looks like the robot slowly rotating even at rest.
        self.gx_bias = 0.0
        self.gy_bias = 0.0
        self.gz_bias = 0.0
        if calib_n > 0:
            self._calibrate_gyro(calib_n)

        # Publishers
        self.imu_pub  = self.create_publisher(Imu,     '/imu', 10)
        self.temp_pub = self.create_publisher(Float32, '/imu/temperature', 10)

        # Timer
        period = 1.0 / rate_hz
        self.create_timer(period, self.timer_cb)
        self.get_logger().info(f'IMU node started at {rate_hz} Hz')

    def _calibrate_gyro(self, n: int):
        self.get_logger().info(
            f'Calibrating gyro bias over {n} samples — keep the robot still…'
        )
        gx_sum = gy_sum = gz_sum = 0.0
        used = 0
        for _ in range(n):
            try:
                _, _, _, _, gx, gy, gz = self._read_all()
            except Exception:
                time.sleep(0.02)
                continue
            gx_sum += gx
            gy_sum += gy
            gz_sum += gz
            used += 1
            time.sleep(0.02)

        if used == 0:
            self.get_logger().error('Gyro calibration failed — no samples read')
            return

        self.gx_bias = gx_sum / used
        self.gy_bias = gy_sum / used
        self.gz_bias = gz_sum / used
        self.get_logger().info(
            f'Gyro bias (rad/s): '
            f'x={self.gx_bias / GYRO_SCALE * DEG_TO_RAD:+.5f}  '
            f'y={self.gy_bias / GYRO_SCALE * DEG_TO_RAD:+.5f}  '
            f'z={self.gz_bias / GYRO_SCALE * DEG_TO_RAD:+.5f}'
        )

    def _init_chip(self):
        # Clear sleep bit — chip wakes up on power-on in sleep mode
        self.bus.write_byte_data(self.addr, REG_PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        # Accel config: ±2g (bits [4:3] = 00)
        self.bus.write_byte_data(self.addr, REG_ACCEL_CFG, 0x00)

        # Gyro config: ±250°/s (bits [4:3] = 00)
        self.bus.write_byte_data(self.addr, REG_GYRO_CFG, 0x00)

        self.get_logger().info('MPU-6500 initialised')

    def _read_word(self, reg):
        high = self.bus.read_byte_data(self.addr, reg)
        low  = self.bus.read_byte_data(self.addr, reg + 1)
        val  = (high << 8) | low
        return val - 65536 if val > 32767 else val

    def _read_all(self):
        # Burst read 14 bytes starting at ACCEL_XOUT (faster, atomic)
        data = self.bus.read_i2c_block_data(self.addr, REG_ACCEL_XOUT, 14)

        def to_signed(h, l):
            v = (h << 8) | l
            return v - 65536 if v > 32767 else v

        ax = to_signed(data[0],  data[1])
        ay = to_signed(data[2],  data[3])
        az = to_signed(data[4],  data[5])
        t  = to_signed(data[6],  data[7])
        gx = to_signed(data[8],  data[9])
        gy = to_signed(data[10], data[11])
        gz = to_signed(data[12], data[13])
        return ax, ay, az, t, gx, gy, gz

    def timer_cb(self):
        try:
            ax, ay, az, t, gx, gy, gz = self._read_all()
        except Exception as e:
            self.get_logger().warn(f'I2C read error: {e}')
            return

        now = self.get_clock().now().to_msg()

        # --- IMU message ---
        msg = Imu()
        msg.header.stamp    = now
        msg.header.frame_id = self.frame_id

        # Orientation unknown — fill covariance with -1 (tells Nav2 to ignore)
        msg.orientation_covariance[0] = -1.0

        # Accelerometer — convert to m/s²
        msg.linear_acceleration.x = (ax / ACCEL_SCALE) * GRAVITY
        msg.linear_acceleration.y = (ay / ACCEL_SCALE) * GRAVITY
        msg.linear_acceleration.z = (az / ACCEL_SCALE) * GRAVITY
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01

        # Gyroscope — subtract calibrated bias, then convert to rad/s
        msg.angular_velocity.x = ((gx - self.gx_bias) / GYRO_SCALE) * DEG_TO_RAD
        msg.angular_velocity.y = ((gy - self.gy_bias) / GYRO_SCALE) * DEG_TO_RAD
        msg.angular_velocity.z = ((gz - self.gz_bias) / GYRO_SCALE) * DEG_TO_RAD
        msg.angular_velocity_covariance[0] = 0.001
        msg.angular_velocity_covariance[4] = 0.001
        msg.angular_velocity_covariance[8] = 0.001

        self.imu_pub.publish(msg)

        # --- Temperature ---
        temp_c = (t / 333.87) + 21.0   # MPU-6500 formula
        temp_msg = Float32()
        temp_msg.data = float(temp_c)
        self.temp_pub.publish(temp_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
