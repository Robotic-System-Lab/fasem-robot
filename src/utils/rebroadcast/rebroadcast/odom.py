import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomRelay(Node):
	def __init__(self):
		super().__init__('odom')
		self.br = tf2_ros.TransformBroadcaster(self)
		self.static_br = tf2_ros.StaticTransformBroadcaster(self)
		self.initialized_transform = False
		self.declare_parameter('source_odom', "/odom")
		self.source_odom = self.get_parameter('source_odom').value
		self.odom_publisher = self.create_publisher(Odometry, '/fasem_odom', 10)

		self.create_subscription(Odometry, self.source_odom, self.callback_odom, 10)
	
	def init_merged_frames(self, msg: Odometry):
		static_fp_to_merged_tf = TransformStamped()
		static_fp_to_merged_tf.header.stamp = msg.header.stamp
		static_fp_to_merged_tf.header.frame_id = 'fasem_footprint'
		static_fp_to_merged_tf.child_frame_id = 'fasem_link'
		static_fp_to_merged_tf.transform.translation.x = msg.pose.pose.position.x
		static_fp_to_merged_tf.transform.translation.y = msg.pose.pose.position.y
		static_fp_to_merged_tf.transform.translation.z = msg.pose.pose.position.z
		static_fp_to_merged_tf.transform.rotation.x = msg.pose.pose.orientation

		static_merged_to_scan_tf = TransformStamped()
		static_merged_to_scan_tf.header.stamp = msg.header.stamp
		static_merged_to_scan_tf.header.frame_id = 'fasem_link'
		static_merged_to_scan_tf.child_frame_id = 'fasem_scan'
		static_merged_to_scan_tf.transform.translation.x = msg.pose.pose.position.x
		static_merged_to_scan_tf.transform.translation.y = msg.pose.pose.position.y
		static_merged_to_scan_tf.transform.translation.z = msg.pose.pose.position.z
		static_merged_to_scan_tf.transform.rotation = msg.pose.pose.orientation

		self.static_br.sendTransform([static_fp_to_merged_tf, static_merged_to_scan_tf])
		self.initialized_transform = True
		self.get_logger().warn('Static footprint initialized.')

	def callback_odom(self, msg: Odometry):
		#############################################
		#### Odom manipulation
		latest_stamp = self.get_clock().now().to_msg()
		save_odom = msg
		save_odom.header.stamp = latest_stamp
		save_odom.header.frame_id = 'fasem_odom'
		save_odom.child_frame_id = 'fasem_footprint'
	
		#############################################
		#### Odom frame setup
		odom_tf = TransformStamped()
		odom_tf.header.stamp = latest_stamp
		odom_tf.header.frame_id = save_odom.header.frame_id
		odom_tf.child_frame_id = save_odom.child_frame_id 
		odom_tf.transform.translation.x = save_odom.pose.pose.position.x
		odom_tf.transform.translation.y = save_odom.pose.pose.position.y
		odom_tf.transform.translation.z = save_odom.pose.pose.position.z
		odom_tf.transform.rotation = save_odom.pose.pose.orientation
	
		#############################################
		#### all transform broadcasted
		self.odom_publisher.publish(save_odom)

def main(args=None):
	rclpy.init(args=args)
	node = OdomRelay()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()