import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class ScandomNode(Node):
	def __init__(self):
		super().__init__('scandom')
		self.br = tf2_ros.TransformBroadcaster(self)
		self.static_br = tf2_ros.StaticTransformBroadcaster(self)
		self.init_merging = False
		self.initialized_transform = False
	
		self.declare_parameter('maxrange', 20.0)
		self.declare_parameter('source_odom', "/odom")
		self.declare_parameter('source_scan', "/scan")

		self.maxrange = self.get_parameter('maxrange').value
		self.source_odom = self.get_parameter('source_odom').value
		self.source_scan = self.get_parameter('source_scan').value

		self.odom_publisher = self.create_publisher(Odometry, '/fasem_odom', 10)
		self.scan_publisher = self.create_publisher(LaserScan, '/fasem_scan', 10)

		self.create_subscription(Odometry, self.source_odom, self.callback_odom, 10)
		self.create_subscription(LaserScan, self.source_scan, self.callback_scan, 10)
	
	def init_merged_frames(self, msg: Odometry):
		static_fp_to_merged_tf = TransformStamped()
		static_fp_to_merged_tf.header.stamp = msg.header.stamp
		static_fp_to_merged_tf.header.frame_id = 'fasem_footprint'
		static_fp_to_merged_tf.child_frame_id = 'fasem_link'
		static_fp_to_merged_tf.transform.translation.x = 0.0
		static_fp_to_merged_tf.transform.translation.y = 0.0
		static_fp_to_merged_tf.transform.translation.z = 0.0
		static_fp_to_merged_tf.transform.rotation.x = 0.0
		static_fp_to_merged_tf.transform.rotation.y = 0.0
		static_fp_to_merged_tf.transform.rotation.z = 0.0
		static_fp_to_merged_tf.transform.rotation.w = 1.0

		static_merged_to_scan_tf = TransformStamped()
		static_merged_to_scan_tf.header.stamp = msg.header.stamp
		static_merged_to_scan_tf.header.frame_id = 'fasem_link'
		static_merged_to_scan_tf.child_frame_id = 'fasem_scan'
		static_merged_to_scan_tf.transform.translation.x = 0.0
		static_merged_to_scan_tf.transform.translation.y = 0.0
		static_merged_to_scan_tf.transform.translation.z = 0.0
		static_merged_to_scan_tf.transform.rotation.x = 0.0
		static_merged_to_scan_tf.transform.rotation.y = 0.0
		static_merged_to_scan_tf.transform.rotation.z = 0.0
		static_merged_to_scan_tf.transform.rotation.w = 1.0

		self.static_br.sendTransform([static_fp_to_merged_tf, static_merged_to_scan_tf])
		self.initialized_transform = True
		self.get_logger().warn('Static footprint initialized.')

	def callback_odom(self, msg: Odometry):
		#############################################
		#### Odom manipulation
		save_odom = msg
		save_odom.header.stamp = msg.header.stamp
		save_odom.header.frame_id = 'fasem_odom'
		save_odom.child_frame_id = 'fasem_footprint'
	
		#############################################
		#### Odom frame setup
		odom_tf = TransformStamped()
		odom_tf.header.stamp = msg.header.stamp
		odom_tf.header.frame_id = save_odom.header.frame_id
		odom_tf.child_frame_id = save_odom.child_frame_id 
		odom_tf.transform.translation.x = save_odom.pose.pose.position.x
		odom_tf.transform.translation.y = save_odom.pose.pose.position.y
		odom_tf.transform.translation.z = save_odom.pose.pose.position.z
		odom_tf.transform.rotation = save_odom.pose.pose.orientation
	
		#############################################
		#### all transform broadcasted
		self.odom_publisher.publish(save_odom)
		self.br.sendTransform(odom_tf)
		if not self.initialized_transform:
			self.init_merged_frames(msg=save_odom)

	def callback_scan(self, msg: LaserScan):
		#############################################
		#### Temporary scan setup
		latest_stamp = msg.header.stamp
		limited_scan = LaserScan()

		#############################################
		#### Scan saved
		limited_scan.header.frame_id = 'fasem_scan'
		limited_scan.angle_min = msg.angle_min
		limited_scan.angle_max = msg.angle_max
		limited_scan.angle_increment = msg.angle_increment
		limited_scan.time_increment = msg.time_increment
		limited_scan.scan_time = msg.scan_time
		limited_scan.range_min = msg.range_min
		limited_scan.range_max = self.maxrange
		limited_scan.ranges = [
			distance if distance <= self.maxrange else float('inf')
			for distance in msg.ranges
		]
		limited_scan.intensities = [
			intensity if msg.ranges[i] <= self.maxrange else 0.0
			for i, intensity in enumerate(msg.intensities)
		] if msg.intensities else []
		limited_scan.header.stamp = latest_stamp
		self.scan_publisher.publish(limited_scan)

def main(args=None):
	rclpy.init(args=args)
	node = ScandomNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()