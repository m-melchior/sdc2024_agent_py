#!/usr/bin/env python3

from geometry_msgs.msg import Quaternion
import math
import numpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleLocalPosition, VehicleStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from scipy.spatial.transform import Rotation as SST_Rotation

HOME_LAT = 487227300
HOME_LONG = 115506793
HOME_ALT = 200

HEIGHT_HOVER = -3.0

POS_INITIAL = [16.625, -2.0, 0.0]

SETPOINTS = [
	[[5.0, -5.0, -3.0], -180.0],
	[[5.0, -15.0, -3.0], -90.0],
	[[16.625, -2.0, -3.0], 0.0]
]

THRESHOLD_DISTANCE = 1.0

STATE_IDLE = 0
STATE_OFFBOARD = 1
STATE_ARM = 2
STATE_TAKEOFF = 3
STATE_MOVE1 = 4
STATE_TURN = 5
STATE_MOVE2 = 6
STATE_LAND = 7

TIMER_STATE_CHANGE = 100

def get_distance(x1, y1, x2, y2):
	dx = x2 - x1
	dy = y2 - y1
	distance = math.sqrt(dx**2 + dy**2)
	return distance

class SDC2024_Agent(Node):
	# ******************************************
	def __init__(self) -> None:
		super().__init__('sdc2024_agent')

		qos_profile = QoSProfile(
			reliability = ReliabilityPolicy.BEST_EFFORT,
			durability = DurabilityPolicy.TRANSIENT_LOCAL,
			history = HistoryPolicy.KEEP_LAST,
			depth = 1
		)

		self.publisher_offboard_control_mode	= self.create_publisher(OffboardControlMode,		'/fmu/in/offboard_control_mode',		qos_profile)
		self.publisher_trajectory_setpoint		= self.create_publisher(TrajectorySetpoint,			'/fmu/in/trajectory_setpoint',			qos_profile)
		self.publisher_vehicle_command			= self.create_publisher(VehicleCommand,				'/fmu/in/vehicle_command',				qos_profile)
		self.publisher_vehicle_visual_odometry	= self.create_publisher(VehicleOdometry,			'/fmu/in/vehicle_visual_odometry',		qos_profile)

		self.subscriber_vehicle_local_position	= self.create_subscription(VehicleLocalPosition,	'/fmu/out/vehicle_local_position',		self.callback_vehicle_local_position,	qos_profile)
		self.subscriber_vehicle_status			= self.create_subscription(VehicleStatus,			'/fmu/out/vehicle_status',				self.vehicle_status_callback,			qos_profile)

		self.counter_offboard_timer = 50
		self.vehicle_local_position = VehicleLocalPosition()
		self.vehicle_status = VehicleStatus()

		self.odometry_position = POS_INITIAL

		self.timer_offboard = self.create_timer(0.1, self.callback_timer)

		# self.timer_odometry = self.create_timer(1 / 30, self.publish_vehicle_odometry)

		self.index_setpoint = 0

		self.state = STATE_IDLE

	# ******************************************
	def callback_vehicle_local_position(self, vehicle_local_position):
		# print(f"vehicle_local_position: {vehicle_local_position}")
		self.vehicle_local_position = vehicle_local_position
		print(f"self.vehicle_local_position.x, self.vehicle_local_position.y: {self.vehicle_local_position.x}, {self.vehicle_local_position.y}")

	# ******************************************
	def vehicle_status_callback(self, vehicle_status):
		self.vehicle_status = vehicle_status

	# ******************************************
	def arm(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1 = 1.0)
		# self.get_logger().info('Arming')

	# ******************************************
	def disarm(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1 = 0.0)
		# self.get_logger().info('DISarming')

	# ******************************************
	def set_mode_offboard(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2 = 6.0)
		# self.get_logger().info("Set Mode OFFBOARD")

	# ******************************************
	def land(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
		# self.get_logger().info("Command LAND")

	# ******************************************
	def publish_offboard_control_heartbeat_signal(self):
		msg = OffboardControlMode()
		msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

		msg.position = True
		msg.velocity = False
		msg.acceleration = False
		msg.attitude = False
		msg.body_rate = False

		self.publisher_offboard_control_mode.publish(msg)

	# ******************************************
	def publish_trajectory_setpoint_position(self, position_in):
		msg = TrajectorySetpoint()
		msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

		msg.position = position_in
		msg.velocity = [float('nan'), float('nan'), float('nan')]
		msg.yaw = self.vehicle_local_position.heading

		# self.get_logger().info(f"position_in {position_in}")

		self.publisher_trajectory_setpoint.publish(msg)

	# ******************************************
	def publish_trajectory_setpoint_velocity(self, velocity_in):
		msg = TrajectorySetpoint()
		msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

		msg.position = [float('nan'), float('nan'), float('nan')]
		msg.velocity = velocity_in
		msg.yaw = self.vehicle_local_position.heading

		# self.get_logger().info(f"velocity_in {velocity_in}")

		self.publisher_trajectory_setpoint.publish(msg)

	# ******************************************
	def publish_trajectory_setpoint_yaw_deg(self, yaw_deg_in):
		msg = TrajectorySetpoint()
		msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

		msg.position = [float('nan'), float('nan'), float('nan')]
		msg.velocity = [float('nan'), float('nan'), float('nan')]
		msg.yaw = math.radians(yaw_deg_in)

		# self.get_logger().info(f"yaw_in {yaw_in}")

		self.publisher_trajectory_setpoint.publish(msg)

	# ******************************************
	def publish_trajectory_setpoint_yaw_rad(self, yaw_rad_in):
		msg = TrajectorySetpoint()
		msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

		msg.position = [float('nan'), float('nan'), float('nan')]
		msg.velocity = [float('nan'), float('nan'), float('nan')]
		msg.yaw = yaw_rad_in


		# self.get_logger().info(f"yaw_in {yaw_in}")

		self.publisher_trajectory_setpoint.publish(msg)

	# ******************************************
	def publish_vehicle_command(self, command, **params) -> None:
		msg = VehicleCommand()
		msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

		msg.command = command
		msg.param1 = params.get("param1", 0.0)
		msg.param2 = params.get("param2", 0.0)
		msg.param3 = params.get("param3", 0.0)
		msg.param4 = params.get("param4", 0.0)
		msg.param5 = params.get("param5", 0.0)
		msg.param6 = params.get("param6", 0.0)
		msg.param7 = params.get("param7", 0.0)
		msg.target_system = 1
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		self.publisher_vehicle_command.publish(msg)

	# ******************************************
	def publish_vehicle_odometry(self) -> None:
		# print(f"self.vehicle_local_position.x, self.vehicle_local_position.y: {self.vehicle_local_position.x}, {self.vehicle_local_position.y}")

		msg = VehicleOdometry()
		msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
		# msg.timestamp_sample = int(self.get_clock().now().nanoseconds / 1000)

		msg.pose_frame = VehicleOdometry.POSE_FRAME_NED

		msg.position = numpy.array(self.odometry_position, dtype = numpy.float32)

		msg.q = numpy.array([float('nan'), float('nan'), float('nan'), float('nan')], dtype = numpy.float32)

		msg.velocity = [float('nan'), float('nan'), float('nan')]

		msg.angular_velocity = [float('nan'), float('nan'), float('nan')]

		msg.position_variance = [float('nan'), float('nan'), float('nan')]
		msg.orientation_variance = [float('nan'), float('nan'), float('nan')]
		msg.velocity_variance = [float('nan'), float('nan'), float('nan')]


		self.publisher_vehicle_visual_odometry.publish(msg)		

	# ******************************************
	def callback_timer(self) -> None:
		self.publish_offboard_control_heartbeat_signal()

		self.counter_offboard_timer += 1
		if (self.counter_offboard_timer == TIMER_STATE_CHANGE):
			self.state += 1
			print(f"self.state: {self.state}")
			self.counter_offboard_timer = 0

		if (self.state== STATE_OFFBOARD):
			self.set_mode_offboard()

		if (self.state == STATE_ARM):
			print("arming")
			self.arm()

		# if (self.state == STATE_TAKEOFF):
		# 	self.publish_trajectory_setpoint_position([float('nan'), float('nan'), HEIGHT_HOVER])
		# 	self.publish_trajectory_setpoint_yaw_deg(90.0)

		# if (self.state == STATE_MOVE1):
		# 	self.publish_trajectory_setpoint_position([float('nan'), float('nan'), HEIGHT_HOVER])
		# 	self.publish_trajectory_setpoint_velocity([0.0, 2.0, 0.0])

		# if (self.state == STATE_TURN):
		# 	yaw_deg = math.degrees(self.vehicle_local_position.heading)
		# 	yaw_deg += 5
		# 	print(f"yaw_deg: {yaw_deg}")
		# 	self.publish_trajectory_setpoint_position([float('nan'), float('nan'), HEIGHT_HOVER])
		# 	self.publish_trajectory_setpoint_yaw_deg(yaw_deg)

		# if (self.state == STATE_MOVE2):
		# 	self.publish_trajectory_setpoint_position([float('nan'), float('nan'), HEIGHT_HOVER])
		# 	self.publish_trajectory_setpoint_velocity([0.0, -2.0, 0.0])

		# if (self.state == STATE_LAND):
		# 	self.land()
		# 	exit(0)


def main(args=None) -> None:
	print('Starting SDC2024 Agent')
	rclpy.init(args=args)

	sdc2024_agent = SDC2024_Agent()

	rclpy.spin(sdc2024_agent)

	sdc2024_agent.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	try:
		main()
	except Exception as e:
		print(e)
