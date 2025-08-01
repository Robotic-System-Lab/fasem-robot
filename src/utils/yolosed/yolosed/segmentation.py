import time
import json
import rclpy
import cv2
import os
import torch
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge
from ultralytics import YOLO

import numpy as np
from .hazard import hazard_lookup

class YOLOSegnetNode(Node):
  def __init__(self):
    super().__init__('segmentation')
    self.get_logger().info('Segmentation node has been started.')
    self.bridge = CvBridge()
    self.segmentation_counter = 0
    
    self.declare_parameter('fov_h', 60.0)
    self.declare_parameter('view_p', 0.25)
    self.declare_parameter('view_h', 0.10)
    self.fov_h = self.get_parameter('fov_h').value
    self.view_p = self.get_parameter('view_p').value
    self.view_h = self.get_parameter('view_h').value
    
    self.declare_parameter('cam_count', 6)
    self.declare_parameter('cam_center', 150)
    self.cam_count = self.get_parameter('cam_count').value
    self.cam_center = self.get_parameter('cam_center').value
    self.angle_default = round(360 / self.cam_count)
    
    self.declare_parameter('segmentation_model', "yolo11m-seg")
    self.segmentation_model = self.get_parameter('segmentation_model').value
    
    self.get_logger().info('Loading Model...')
    model_path = os.path.join(os.path.dirname(__file__), 'model', f"{self.segmentation_model}")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    self.model = YOLO(model_path)
    self.model.to(device)
    self.get_logger().info(f'Model loaded on: {self.model.device}, ready to perform segmentation.')
    
    self.timestamp = 0
    self.images = [None] * self.cam_count
    self.deg360 = [{
                'label': None,
                'conf': 0,
              }] * 360
    
    self.subscribers = []
    self.label_start_publisher = self.create_publisher(Float64, '/label_start', 10)
    self.label_end_publisher = self.create_publisher(String, '/label_end', 10)
    for i in range(self.cam_count):
      topic_name = f'/camera_{i + 1}/image_raw'
      self.subscribers.append(
        self.create_subscription(
          Image,
          topic_name,
          lambda msg, idx=i: self.image_callback(msg, idx),
          10
        )
      )
    # self.timer = self.create_timer(0.1, self.display_images)
  
  def image_callback(self, msg, index):
    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    self.images[index] = cv_image

  def collect_results(self, results, cv_image, index):
    label_data = []
    segmentation_data = []
    height = cv_image.shape[0]
    width = cv_image.shape[1]
    pixels_per_degree = width//round(self.fov_h)
    
    gaps = self.angle_default - self.fov_h
    inner_gap = round(gaps / 2) if gaps > 0 else 0
    outer_gap = 0 if gaps > 0 else abs(round(gaps / 2))
    offset_pixel = round(outer_gap * pixels_per_degree)
    
    x_min_valid = offset_pixel
    x_max_valid = width - offset_pixel
    y_min_valid = int(self.view_p * height)
    y_max_valid = int(self.view_h * height + y_min_valid)
    
    for result in results:
      if result.boxes is not None:
        for box in result.boxes:
          xyxy = box.xyxy[0].tolist()
          x1, _, x2, _ = xyxy
          class_id = box.cls.item()
          label_name = self.model.names[int(class_id)]
          conf = box.conf.item()

          label_data.append({
            'label': hazard_lookup.get(label_name, 1),
            'conf': conf,
            'name': label_name,
          })
      if result.masks is not None:
        idx = 0
        for mask in result.masks.xy:
          valid_points = mask[
            (mask[:, 1] >= y_min_valid) &
            (mask[:, 1] <= y_max_valid) &
            (mask[:, 0] >= x_min_valid) &
            (mask[:, 0] <= x_max_valid)
          ]
          if valid_points.size > 0:
            x1 = int(valid_points[:, 0].min())
            x2 = int(valid_points[:, 0].max())
            segmentation_data.append({
              'x1': x1,
              'x2': x2,
              'conf': label_data[idx]['conf'],
              'label': label_data[idx]['label'],
              'name': label_data[idx]['name'],
            })
          else:
            segmentation_data.append({'name': f"INVALID-{label_data[idx]['name']}"})
          idx += 1
    
    for data in segmentation_data:
      # self.get_logger().info(f"Collected data: {data['name']}")
      if 'label' not in data:
        continue
      for degree in range(
        (index * self.angle_default) + inner_gap, 
        ((index+1) * self.angle_default) - inner_gap
      ):
        if data['x1'] <= (degree-(self.angle_default*index))*pixels_per_degree < data['x2']:
          if (self.deg360[degree]['conf'] == 0 or self.deg360[degree]['conf'] < data['conf']):
            self.deg360[degree] = {
              'label': data['label'],
              'conf': data['conf'],
              'x1': data['x1'],
              'x2': data['x2']
            }

  def segment_image(self, image, index):
    """Gunakan segNet untuk segmentasi gambar."""
    results = self.model(image, verbose=False)
    segmented_image = results[0].plot()
    self.collect_results(results, image, index)
    return segmented_image

  def display_images(self):
    """Gabungkan dan tampilkan gambar dari semua kamera dengan hasil segmentasi."""
    if all(image is not None for image in self.images):
      try:
        # self.get_logger().info("Performing segmentation on collected images...")
        self.segmentation_counter += 1
        self.timestamp = time.time()
        label_start_msg = Float64()
        label_start_msg.data = self.timestamp
        self.label_start_publisher.publish(label_start_msg)
        
        self.deg360 = [{'label': None, 'conf': 0}] * 360
        collected_images = [self.segment_image(image, idx) for idx, image in enumerate(self.images)]
        resized_images = []
        target_height = 160
        for image in collected_images:
            h, w = image.shape[:2]
            scale = target_height / h if h > target_height else 1
            new_w = int(w * scale)
            resized_image = cv2.resize(image, (new_w, target_height))
            resized_images.append(resized_image)

        border_thickness = 5
        bordered_images = []
        for idx, img in enumerate(resized_images):
            bordered_images.append(img)
            if idx < len(resized_images) - 1:
                # Membuat kolom border dengan warna hitam (0, 0, 0)
                border = np.full((target_height, border_thickness, 3), 0, dtype=np.uint8)
                bordered_images.append(border)

        # Gabungkan semua gambar dan border secara horizontal
        combined_image = cv2.hconcat(bordered_images)

        # Tambahkan overlay hitam semi transparan (opacity 50%)
        overlay = combined_image.copy()
        h, w = combined_image.shape[:2]
        alpha = 0.5
        top_end = int(self.view_p * h)
        bottom_start = int((self.view_h + self.view_p) * h)
        cv2.rectangle(overlay, (0, 0), (w, top_end), (0, 0, 0), -1)
        cv2.rectangle(overlay, (0, bottom_start), (w, h), (0, 0, 0), -1)

        # Gabungkan overlay dengan combined_image
        combined_image = cv2.addWeighted(overlay, alpha, combined_image, 1 - alpha, 0)

        cv2.imshow("Segmented Images", combined_image)
        cv2.waitKey(1)

        detected = [
          self.deg360[i]['label']
          for i in range(360)
        ]
        reversed_detected = detected[::-1]
        translate_detected = [
          (reversed_detected[((self.cam_center) + i) % 360])
          if reversed_detected[((self.cam_center) + i) % 360] is not None else 99
          for i in range(360)
        ]
        translate_detected = [100 for _ in translate_detected]
        payload = {
          'count': self.segmentation_counter,
          'timestamp': self.timestamp,
          'detected': translate_detected
        }
        msg_out = String()
        msg_out.data = json.dumps(payload)
        self.label_end_publisher.publish(msg_out)
        self.get_logger().info(f"Successfully performed segmentation for counter: {self.segmentation_counter}")
      except Exception as e:
        self.get_logger().error(f"Error: {e}")
      self.images = [None] * self.cam_count
    # else:
    #   self.get_logger().warning("Not all camera feeds are available.")
    
def main(args=None):
  rclpy.init(args=args)
  node = YOLOSegnetNode()
  try:
    while rclpy.ok():
      # Proses callback subscription
      rclpy.spin_once(node, timeout_sec=0.1)
      # Panggil display_images secara berurutan setelah callback selesai
      node.display_images()
  except KeyboardInterrupt:
    node.get_logger().info('KeyboardInterrupt, shutting down node.')
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()