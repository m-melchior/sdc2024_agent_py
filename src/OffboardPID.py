import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
import time


class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self._prev_error = 0.0
        self._integral = 0.0

    def update(self, current_value, dt):
        error = self.setpoint - current_value
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt
        output = (self.kp * error) + (self.ki * self._integral) + (self.kd * derivative)
        self._prev_error = error
        return output

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        self.command_publisher = self.create_publisher(VehicleCommand, 'VehicleCommand_PubSubTopic', 10)
        self.control_mode_publisher = self.create_publisher(OffboardControlMode, 'OffboardControlMode_PubSubTopic', 10)
        self.setpoint_publisher = self.create_publisher(TrajectorySetpoint, 'TrajectorySetpoint_PubSubTopic', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.set_offboard_mode()
        self.arm()
        self.current_setpoint = TrajectorySetpoint()
        self.pid_x = PID(1.0, 0.0, 0.0)
        self.pid_y = PID(1.0, 0.0, 0.0)
        self.pid_z = PID(1.0, 0.0, 0.0)
        self.pid_yaw = PID(1.0, 0.0, 0.0)

    def timer_callback(self):
        self.publish_control_mode()
        self.publish_velocity_setpoint()

    def set_offboard_mode(self):
        offboard_cmd = VehicleCommand()
        offboard_cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        offboard_cmd.param1 = 1.0  # custom mode
        offboard_cmd.param2 = 6.0  # offboard
        self.command_publisher.publish(offboard_cmd)
        self.get_logger().info('Set to offboard mode')

    def arm(self):
        arm_cmd = VehicleCommand()
        arm_cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        arm_cmd.param1 = 1.0  # arm
        self.command_publisher.publish(arm_cmd)
        self.get_logger().info('Armed')

    def publish_control_mode(self):
        control_mode = OffboardControlMode()
        control_mode.position = False
        control_mode.velocity = True
        control_mode.acceleration = False
        control_mode.attitude = False
        control_mode.body_rate = False
        self.control_mode_publisher.publish(control_mode)
        self.get_logger().info('Publishing control mode')

    def publish_velocity_setpoint(self):
        current_position = self.get_current_position()  # Implement this method to get current position
        dt = 0.1  # Time step
        self.current_setpoint.vx = self.pid_x.update(current_position[0], dt)
        self.current_setpoint.vy = self.pid_y.update(current_position[1], dt)
        self.current_setpoint.vz = self.pid_z.update(current_position[2], dt)
        self.current_setpoint.yaw = self.pid_yaw.update(current_position[3], dt)
        self.setpoint_publisher.publish(self.current_setpoint)
        self.get_logger().info('Publishing velocity setpoint')

    def get_current_position(self):
        # Replace this with actual sensor data or estimation
        return [0.0, 0.0, 0.0, 0.0]  # [x, y, z, yaw]

    def move_to_position(self, x, y, z, yaw):
        self.pid_x.setpoint = x
        self.pid_y.setpoint = y
        self.pid_z.setpoint = z
        self.pid_yaw.setpoint = yaw

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    
    # Example movement to a specific position
    offboard_control.move_to_position(1.0, 1.0, -1.0, 0.0)
    
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
