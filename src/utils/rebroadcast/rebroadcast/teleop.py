import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopRelay(Node):
	def __init__(self):
		super().__init__('teleop')
		self.initialized_transform = False
		self.vel_publisher = self.create_publisher(Twist, '/a200_1060/joy_teleop/cmd_vel', 10)
		self.create_subscription(Twist, "/cmd_vel", self.callback_vel, 10)
	
	def callback_vel(self, msg:Twist):
		copied_msg = msg
		self.vel_publisher.publish(copied_msg)

def main(args=None):
	rclpy.init(args=args)
	node = TeleopRelay()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()