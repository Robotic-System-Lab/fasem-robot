import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScandomNode(Node):
	def __init__(self):
		super().__init__('scan')
		self.initialized_transform = False
		self.declare_parameter('max_scan', 20.0)
		self.declare_parameter('source_scan', "/scan")
		self.max_scan = self.get_parameter('max_scan').value
		self.source_scan = self.get_parameter('source_scan').value
		self.publisher_scan = self.create_publisher(LaserScan, '/fasem_scan', 10)
		self.create_subscription(LaserScan, self.source_scan, self.callback_scan, 10)
	
	def callback_scan(self, msg: LaserScan):
		#############################################
		#### Temporary scan setup
		latest_stamp = self.get_clock().now().to_msg()
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
		limited_scan.range_max = self.max_scan
		limited_scan.ranges = [
			distance if distance <= self.max_scan else float('inf')
			for distance in msg.ranges
		]
		limited_scan.header.stamp = latest_stamp
		self.publisher_scan.publish(limited_scan)

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