import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from amr_msgs.msg import WheelMotor
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import JointState

from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
from serial_comm.motor_driver import MotorDriver


class Nodelet(Node):
    def __init__(self):
        super().__init__('base_main')
        self.pub = self.create_publisher(WheelMotor, '/wheelmotor', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 100)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 100)

        self.amr_data_distance = self.create_publisher(String, '/amr_data_distance', 10)

        self.dt = 0.02
        self.timer_ = self.create_timer(self.dt, self.timer_callback)
        self.tf_broadcaster = TransformBroadcaster(self)

        # global coordinate (odom -> map)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.loopcnt = 0

        self.firstloop = True
        self.JOY_CONTROL = False

        # Motor driver class
        self.md = MotorDriver()

        # PID related variables
        self.p_gain = 1.0
        self.i_gain = 0.0
        self.d_gain = 0.01
        self.forget = 0.99

        self.err1_prev, self.err1_i = 0.0, 0.0
        self.err2_prev, self.err2_i = 0.0, 0.0
        self.torque1, self.torque2 = 0, 0
        self.velocity1, self.velocity2 = 0, 0

        # target position (encoder)
        self.target_pos1, self.target_pos2 = 0, 0

        # joy gain
        self.joy_fb = 0.0
        self.joy_lr = 0.0
        self.v_gain = 100.0
        self.w_gain = 50.0

        self.joy_r2 = 0.0
        self.joy_l2 = 0.0
        self.change_mode = 0
        self.joy_stop = 0

        self.joy_speed_up = 0
        self.joy_speed_down = 0
        self.joy_speed_up_old = 0
        self.joy_speed_down_old = 0
        self.gain_count = 2
        self.gain_list = [0.5, 0.7, 1.0, 1.3, 1.5, 1.7, 2.0, 2.3, 2.5, 4.0]

        self.msg_wheelmotor = WheelMotor()

        # odom param
        self.wheel_diameter = 0.13
        # 만약 "좌우 16cm"가 각 휠까지 거리라면 → wheel_separation = 0.32
        self.wheel_separation = 0.32
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        self.last_time = self.get_clock().now()

        # accumulated distance
        self.accumulated_distance = 0.0

        # Encoder positions
        self.last_pos1 = 0.0
        self.last_pos2 = 0.0
        self.cur_pos1 = 0.0
        self.cur_pos2 = 0.0
        self.del_pos1 = 0.0
        self.del_pos2 = 0.0

        # joint state publisher (for wheel TF in RViz)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.wheel_left_angle  = 0.0  # rad
        self.wheel_right_angle = 0.0  # rad


        # cmd_vel param
        self.cmd_vel_r = 0.0
        self.cmd_vel_l = 0.0
        self.cmd_vel_r_old = 0.0
        self.cmd_vel_l_old = 0.0
        self.gear_ratio = 4.33
        self.linear_velocity_old = 0.0
        self.angular_velocity_old = 0.0

        self.vel_input1 = 0.0
        self.vel_input2 = 0.0
        self.vel_input1_old = 0.0
        self.vel_input2_old = 0.0

        # lowpass filter
        self.v_motor_last = 0.0
        self.w_motor_last = 0.0
        self.alpha = 0.5

    def timer_callback(self):
        self.loopcnt += 1

        if self.firstloop:
            # 초기 상태 읽기
            self.md.send_vel_cmd(self.velocity1, self.velocity2)
            self.md.recv_motor_state()
            self.target_pos1 = self.md.pos1
            self.target_pos2 = self.md.pos2

            self.del_pos1 = self.md.pos1
            self.del_pos2 = self.md.pos2

            self.firstloop = False
            return

        # 1) 제어 명령 전송
        if self.JOY_CONTROL:
            # Joystick → 속도 제어
            self.vel_input1 = self.v_gain * self.joy_fb - self.w_gain * self.joy_lr
            self.vel_input2 = self.v_gain * self.joy_fb + self.w_gain * self.joy_lr

            self.vel_input1 = self.Lowpass_filter(self.vel_input1, self.vel_input1_old, 0.1)
            self.vel_input2 = self.Lowpass_filter(self.vel_input2, self.vel_input2_old, 0.1)

            self.vel_input1_old = self.vel_input1
            self.vel_input2_old = self.vel_input2

            if self.joy_stop == 1:
                # 현재 위치로 position hold
                self.md.send_position_cmd(self.md.pos1, self.md.pos2, int(60), int(60))
                self.get_logger().info('stop')
            else:
                # 속도 제어 명령
                self.md.send_vel_cmd(self.vel_input1, self.vel_input2)

            self.msg_wheelmotor.target1 = int(self.vel_input1)
            self.msg_wheelmotor.target2 = int(self.vel_input2)

        else:
            # AUTO 모드: cmd_vel → 엔코더 target_pos 적분
            self.md.send_position_cmd(
                int(self.target_pos1),
                int(self.target_pos2),
                int(60 * self.gear_ratio),
                int(60 * self.gear_ratio)
            )

            self.msg_wheelmotor.target1 = int(self.target_pos1)
            self.msg_wheelmotor.target2 = int(self.target_pos2)

        # 2) 모터 상태 수신
        self.md.recv_motor_state()

        self.msg_wheelmotor.position1 = self.md.pos1
        self.msg_wheelmotor.position2 = self.md.pos2
        self.msg_wheelmotor.velocity1 = self.md.rpm1
        self.msg_wheelmotor.velocity2 = self.md.rpm2
        self.msg_wheelmotor.current1 = int(self.md.current1)
        self.msg_wheelmotor.current2 = int(self.md.current2)
        self.msg_wheelmotor.v_x = (self.md.rpm1 + self.md.rpm2) * np.pi * self.wheel_diameter / (60 * 2 * self.gear_ratio)
        self.msg_wheelmotor.w_z = (self.md.rpm2 - self.md.rpm1) * np.pi * self.wheel_diameter / (60 * self.wheel_separation * self.gear_ratio)

        self.pub.publish(self.msg_wheelmotor)

        # 3) Odom 계산
        self.cur_pos1 = self.md.pos1 - self.del_pos1
        self.cur_pos2 = self.md.pos2 - self.del_pos2

        delta_pos1 = self.cur_pos1 - self.last_pos1
        delta_pos2 = self.cur_pos2 - self.last_pos2

        self.last_pos1 = self.cur_pos1
        self.last_pos2 = self.cur_pos2

        left_wheel_disp = (delta_pos1 / self.md.encoder_gain) * (np.pi * self.wheel_diameter)
        right_wheel_disp = (delta_pos2 / self.md.encoder_gain) * (np.pi * self.wheel_diameter)

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 3-1) 바퀴 회전각(rad) 업데이트 (distance / radius)
        radius = self.wheel_diameter / 2.0
        if radius > 0.0:
            self.wheel_left_angle  += left_wheel_disp  / radius
            self.wheel_right_angle += right_wheel_disp / radius

        # 3-2) JointState publish → robot_state_publisher가 wheel TF 갱신
        js = JointState()
        js.header.stamp = current_time.to_msg()
        js.name = ['wheel_left_joint', 'wheel_right_joint']
        js.position = [self.wheel_left_angle, self.wheel_right_angle]
        # velocity, effort는 필요 없으면 생략 가능
        self.joint_pub.publish(js)

        linear_velocity = (left_wheel_disp + right_wheel_disp) / (2.0 * dt)
        angular_velocity = (right_wheel_disp - left_wheel_disp) / (self.wheel_separation * dt)

        self.pose_x += linear_velocity * np.cos(self.pose_theta) * dt
        self.pose_y += linear_velocity * np.sin(self.pose_theta) * dt
        self.pose_theta += angular_velocity * dt

        if self.pose_theta > np.pi:
            self.pose_theta -= 2 * np.pi
        if self.pose_theta < -np.pi:
            self.pose_theta += 2 * np.pi

        self.accumulated_distance += np.fabs(linear_velocity) * dt
        amr_data_distance_ = String()
        amr_msg_string = f"{self.accumulated_distance:.3f} (m)"
        amr_data_distance_.data = amr_msg_string
        self.amr_data_distance.publish(amr_data_distance_)

        # 4) Odom 메시지 publish
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.pose_x
        odom_msg.pose.pose.position.y = self.pose_y

        q = quaternion_from_euler(0, 0, self.pose_theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom_msg)

        # 5) TF(odom -> base_link) broadcast
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.pose_x
        transform.transform.translation.y = self.pose_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(transform)

    def transform_pose_to_map(self, x, y, theta, transform):
        tx = transform.translation.x
        ty = transform.translation.y

        q = transform.rotation
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        transformation_matrix = np.array([
            [np.cos(yaw), -np.sin(yaw), tx],
            [np.sin(yaw),  np.cos(yaw), ty],
            [0,           0,           1]
        ])

        pose_odom = np.array([x, y, 1.0])
        pose_map = np.dot(transformation_matrix, pose_odom)

        theta_map = theta + yaw
        if theta_map > np.pi:
            theta_map -= 2 * np.pi
        if theta_map < -np.pi:
            theta_map += 2 * np.pi

        return pose_map[0], pose_map[1], theta_map

    def cmd_vel_callback(self, msg):
        if not self.JOY_CONTROL:
            linear_velocity = msg.linear.x
            angular_velocity = msg.angular.z
            control_dt = 0.05

            velocity_right = (linear_velocity + (self.wheel_separation / 2.0) * angular_velocity)
            velocity_left = (linear_velocity - (self.wheel_separation / 2.0) * angular_velocity)

            encoder_delta_right = (velocity_right * control_dt * self.md.encoder_gain) / (np.pi * self.wheel_diameter)
            encoder_delta_left = (velocity_left * control_dt * self.md.encoder_gain) / (np.pi * self.wheel_diameter)

            self.target_pos1 += encoder_delta_left
            self.target_pos2 += encoder_delta_right

    def position_control(self, target1, target2):
        err1 = target1 - self.md.pos1
        self.err1_i = self.forget * (self.err1_i + err1 * self.dt)
        err1_d = (err1 - self.err1_prev) / self.dt
        self.err1_prev = err1

        self.velocity1 = self.p_gain * err1
        self.velocity1 += self.i_gain * self.err1_i
        self.velocity1 += self.d_gain * err1_d

        if self.velocity1 > 1022:
            self.velocity1 = 1022
        elif self.velocity1 < -1022:
            self.velocity1 = -1022
        self.velocity1 = np.array(self.velocity1, dtype=np.int16)

        err2 = target2 - self.md.pos2
        self.err2_i = self.forget * (self.err2_i + err2 * self.dt)
        err2_d = (err2 - self.err2_prev) / self.dt
        self.err2_prev = err2

        self.velocity2 = self.p_gain * err2
        self.velocity2 += self.i_gain * self.err2_i
        self.velocity2 += self.d_gain * err2_d

        if self.velocity2 > 1022:
            self.velocity2 = 1022
        elif self.velocity2 < -1022:
            self.velocity2 = -1022
        self.velocity2 = np.array(self.velocity2, dtype=np.int16)

    def joy_callback(self, msg):
        self.joy_fb = msg.axes[1]
        self.joy_lr = msg.axes[2]

        self.joy_r2 = msg.axes[4]
        self.joy_l2 = msg.axes[5]
        self.joy_stop = msg.buttons[0]

        self.joy_speed_up = msg.buttons[11]
        self.joy_speed_down = msg.buttons[15]

        if self.gain_count < len(self.gain_list) - 1 and self.joy_speed_up == 1 and self.joy_speed_up_old == 0:
            self.gain_count += 1
            self.get_logger().info(f'speed_up : {self.gain_list[self.gain_count]}')

        if self.gain_count > 0 and self.joy_speed_down == 1 and self.joy_speed_down_old == 0:
            self.gain_count -= 1
            self.get_logger().info(f'speed_up : {self.gain_list[self.gain_count]}')

        self.joy_speed_up_old = self.joy_speed_up
        self.joy_speed_down_old = self.joy_speed_down
        self.joy_fb = self.joy_fb * self.gain_list[self.gain_count]

        EPSILON = 1e-5

        if abs(self.joy_r2 + 1.0) < EPSILON and abs(self.joy_l2 + 1.0) < EPSILON and self.change_mode == 1:
            self.change_mode = 0
            self.target_pos1 = self.md.pos1
            self.target_pos2 = self.md.pos2
            self.vel_input1 = 0.0
            self.vel_input2 = 0.0

            self.JOY_CONTROL = not self.JOY_CONTROL
            self.get_logger().info(
                '!!!!!!!!!!!Joystick_control!!!!!!!!!!!!' if self.JOY_CONTROL else '!!!!!!!!!!!!AUTO!!!!!!!!!!!!'
            )
        elif abs(self.joy_r2 - 1.0) < EPSILON and abs(self.joy_l2 - 1.0) < EPSILON:
            self.change_mode = 1

    def Lowpass_filter(self, vel_input, vel_input_1, alpha):
        return alpha * vel_input + (1 - alpha) * vel_input_1


def main(args=None):
    rclpy.init(args=args)
    node = Nodelet()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
