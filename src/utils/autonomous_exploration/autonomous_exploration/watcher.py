import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyWatcher(Node):
	def __init__(self):
		super().__init__('watcher')
		self.subscription = self.create_subscription(
			Joy,
			'/a200_1060/joy_teleop/joy',
			self.joy_callback,
			10
		)
		self.last_b = None

	def joy_callback(self, msg: Joy):
		for i in range(4):
			button_pressed = msg.buttons[i] == 1 if len(msg.buttons) > i else False
			if button_pressed != getattr(self, f'last_button_{i}', None):
				setattr(self, f'last_button_{i}', button_pressed)
				status = "pressed" if button_pressed else "released"
				self.get_logger().info(f"Button {i} is {status}")

def main(args=None):
	rclpy.init(args=args)
	node = JoyWatcher()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()