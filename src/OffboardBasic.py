import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint

class OffboardControl(Node):
	def __init__(self):
		super().__init__('offboard_control')

		qos_profile = QoSProfile(
			reliability = ReliabilityPolicy.BEST_EFFORT,
			durability = DurabilityPolicy.TRANSIENT_LOCAL,
			history = HistoryPolicy.KEEP_LAST,
			depth = 1
		)

		self.publisher_offboard_control_mode	= self.create_publisher(OffboardControlMode,	'/fmu/in/offboard_control_mode',	qos_profile)
		self.vehicle_command_publisher			= self.create_publisher(VehicleCommand,			'/fmu/in/vehicle_command',			qos_profile)
		self.publisher_trajectory_setpoint		= self.create_publisher(TrajectorySetpoint,		'/fmu/in/trajectory_setpoint',		qos_profile)

		self.timer = self.create_timer(0.1, self.callback_timer)

		self.current_setpoint = TrajectorySetpoint()

		self.counter = 0

	def set_offboard_mode(self):
		self.get_logger().info('setting offboard')
		offboard_cmd = VehicleCommand()
		offboard_cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
		offboard_cmd.param1 = 1.0
		offboard_cmd.param2 = 6.0
		self.vehicle_command_publisher.publish(offboard_cmd)
	
	def arm(self):
		self.get_logger().info('arming')
		arm_cmd = VehicleCommand()
		arm_cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
		arm_cmd.param1 = 1.0
		self.vehicle_command_publisher.publish(arm_cmd)

	def publish_control_mode(self):
		msg = OffboardControlMode()
		msg.position = False
		msg.velocity = True
		msg.acceleration = False
		msg.attitude = False
		msg.body_rate = False
		msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
		self.publisher_offboard_control_mode.publish(msg)

	def publish_velocity_setpoint(self):
		self.publisher_trajectory_setpoint.publish(self.current_setpoint)

	def move_forward(self, speed=1.0):
		self.current_setpoint.velocity[0] = speed
		self.current_setpoint.velocity[1] = 0.0
		self.current_setpoint.velocity[2] = 0.0
		# self.current_setpoint.yaw = 0.0

	def move_backward(self, speed=1.0):
		self.current_setpoint.velocity[0] = -speed
		self.current_setpoint.velocity[1] = 0.0
		self.current_setpoint.velocity[2] = 0.0
		# self.current_setpoint.yaw = 0.0

	def move_left(self, speed=1.0):
		self.current_setpoint.velocity[0] = 0.0
		self.current_setpoint.velocity[1] = speed
		self.current_setpoint.velocity[2] = 0.0
		# self.current_setpoint.yaw = 0.0

	def move_right(self, speed=1.0):
		self.current_setpoint.velocity[0] = 0.0
		self.current_setpoint.velocity[1] = -speed
		self.current_setpoint.velocity[2] = 0.0
		# self.current_setpoint.yaw = 0.0

	def move_up(self, speed=1.0):
		self.current_setpoint.velocity[0] = 0.0
		self.current_setpoint.velocity[1] = 0.0
		self.current_setpoint.velocity[2] = speed

	def move_down(self, speed=1.0):
		self.current_setpoint.velocity[0] = 0.0
		self.current_setpoint.velocity[1] = 0.0
		self.current_setpoint.velocity[2] = -speed

	def turn_left(self, yaw_rate=0.1):
		self.current_setpoint.yaw += yaw_rate

	def turn_right(self, yaw_rate=0.1):
		self.current_setpoint.yaw -= yaw_rate

	def move_to(self, position_in):
		self.current_setpoint.position = position_in

	def stop(self):
		self.current_setpoint.position = [0, 0, 0]
		self.velocity = [0, 0, 0]
		self.acceleration = [0, 0, 0]
		self.jerk = [0, 0, 0]

	def callback_timer(self) -> None:
		self.counter += 1

		if (self.counter > 50 and self.counter < 100):
			self.set_offboard_mode()
			self.arm()
		if (self.counter == 100):
			self.move_up()
		if (self.counter == 200):
			self.turn_left()
		if (self.counter == 300):
			self.move_forward()
		if (self.counter == 400):
			self.turn_left()
		if (self.counter == 500):
			self.move_forward()

		self.publish_control_mode()
		self.publish_velocity_setpoint()

def main(args=None):
	rclpy.init(args=args)
	offboard_control = OffboardControl()
	
	rclpy.spin(offboard_control)

	offboard_control.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
