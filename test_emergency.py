#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time

class EmergencyTester(Node):
    def __init__(self):
        super().__init__('emergency_tester')
        self.publisher = self.create_publisher(Joy, '/a200_1060/joy_teleop/joy', 10)
        
    def send_emergency_stop(self):
        """Send emergency stop command (button[2] = 1)"""
        msg = Joy()
        msg.buttons = [0, 0, 1]  # button[2] = 1
        msg.axes = []
        self.publisher.publish(msg)
        print("Emergency STOP sent!")
        
    def send_resume(self):
        """Send resume command (button[1] = 1)"""
        msg = Joy()
        msg.buttons = [0, 1, 0]  # button[1] = 1
        msg.axes = []
        self.publisher.publish(msg)
        print("Resume exploration sent!")
        
    def send_neutral(self):
        """Send neutral state (all buttons = 0)"""
        msg = Joy()
        msg.buttons = [0, 0, 0]  # all buttons released
        msg.axes = []
        self.publisher.publish(msg)

def main():
    rclpy.init()
    tester = EmergencyTester()
    
    try:
        print("Emergency Control Tester")
        print("Commands:")
        print("1. Send Emergency Stop (button[2])")
        print("2. Send Resume (button[1])")
        print("3. Send Neutral (all buttons released)")
        print("q. Quit")
        
        while True:
            cmd = input("\nEnter command: ").strip().lower()
            
            if cmd == '1':
                tester.send_emergency_stop()
                time.sleep(0.1)  # Small delay
                tester.send_neutral()  # Release button
            elif cmd == '2':
                tester.send_resume()
                time.sleep(0.1)  # Small delay  
                tester.send_neutral()  # Release button
            elif cmd == '3':
                tester.send_neutral()
            elif cmd == 'q':
                break
            else:
                print("Invalid command!")
                
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
