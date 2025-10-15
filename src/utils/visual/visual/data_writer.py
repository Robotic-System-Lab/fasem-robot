import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import os
import json
import datetime
import threading
import queue


class DataWriterNode(Node):
    def __init__(self):
        super().__init__('data_writer_node')
        
        # Initialize timestamp for session folder
        self.start_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        
        # Create base directory for saving data
        self.base_path = os.path.join(os.path.expanduser("~"), "fasem", "txt_map", self.start_time)
        os.makedirs(self.base_path, exist_ok=True)
        
        # Queue for asynchronous file writing
        self.write_queue = queue.Queue()
        
        # Start background writer thread
        self.writer_thread = threading.Thread(target=self._writer_worker, daemon=True)
        self.writer_thread.start()
        
        # Subscribe to /map topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.get_logger().info(f'Data writer node started. Saving to: {self.base_path}')
    
    def map_callback(self, msg):
        """Callback for receiving OccupancyGrid messages"""
        # Generate timestamp for this specific data
        timestamp_now = datetime.datetime.now().strftime("%Y%m%d%H%M%S%f")[:-3]  # Include milliseconds
        
        # Prepare data to be saved
        data_to_save = {
            'timestamp': timestamp_now,
            'header': {
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec
                },
                'frame_id': msg.header.frame_id
            },
            'info': {
                'map_load_time': {
                    'sec': msg.info.map_load_time.sec,
                    'nanosec': msg.info.map_load_time.nanosec
                },
                'resolution': msg.info.resolution,
                'width': msg.info.width,
                'height': msg.info.height,
                'origin': {
                    'position': {
                        'x': msg.info.origin.position.x,
                        'y': msg.info.origin.position.y,
                        'z': msg.info.origin.position.z
                    },
                    'orientation': {
                        'x': msg.info.origin.orientation.x,
                        'y': msg.info.origin.orientation.y,
                        'z': msg.info.origin.orientation.z,
                        'w': msg.info.origin.orientation.w
                    }
                }
            },
            'data': list(msg.data)  # Convert to list for JSON serialization
        }
        
        # Add to queue for asynchronous writing
        filename = f"{timestamp_now}.json"
        filepath = os.path.join(self.base_path, filename)
        
        try:
            self.write_queue.put((filepath, data_to_save), block=False)
            self.get_logger().info(f'Queued data for writing: {filename}')
        except queue.Full:
            self.get_logger().warning('Write queue is full, dropping data')
    
    def _writer_worker(self):
        """Background worker thread for writing files"""
        while True:
            try:
                # Get data from queue (blocks until available)
                filepath, data = self.write_queue.get(timeout=1.0)
                
                # Write data to JSON file
                with open(filepath, 'w') as f:
                    json.dump(data, f, indent=2)
                
                self.get_logger().info(f'Saved data to: {os.path.basename(filepath)}')
                
                # Mark task as done
                self.write_queue.task_done()
                
            except queue.Empty:
                # No data available, continue loop
                continue
            except Exception as e:
                self.get_logger().error(f'Error writing file: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = DataWriterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()