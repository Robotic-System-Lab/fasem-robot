#!/usr/bin/env python3
"""
Example subscriber for SegmentationResult custom message type.
Ini adalah contoh bagaimana menggunakan custom message yang baru dibuat.
"""

import rclpy
from rclpy.node import Node
from segmentation_interfaces.msg import SegmentationResult

class SegmentationResultSubscriber(Node):
    def __init__(self):
        super().__init__('segmentation_subscriber_example')
        self.subscription = self.create_subscription(
            SegmentationResult,
            '/segmentation_result',
            self.segmentation_callback,
            10
        )
        self.get_logger().info('SegmentationResult subscriber has been started.')

    def segmentation_callback(self, msg):
        """
        Callback untuk menerima data segmentasi.
        
        Args:
            msg (SegmentationResult): Message berisi data segmentasi
        """
        self.get_logger().info(f"Received segmentation data:")
        self.get_logger().info(f"  Count: {msg.count}")
        self.get_logger().info(f"  Timestamp: {msg.timestamp}")
        self.get_logger().info(f"  Detected objects count: {len(msg.detected)}")
        
        # Contoh analisis data deteksi
        non_empty_detections = [d for d in msg.detected if d not in [99, 100]]
        if non_empty_detections:
            self.get_logger().info(f"  Found {len(non_empty_detections)} hazard detections")
        else:
            self.get_logger().info("  No hazards detected")

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationResultSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
