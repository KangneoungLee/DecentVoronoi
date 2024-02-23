# ROS 2 for bisenet  
# Author:
# - Kangneoung Lee
  

import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Float64, Int8
import cv2
from sensor_msgs.msg import NavSatFix, Image
from cv_bridge import CvBridge
import shutil

import os
import math
import time

import numpy as np



class DecentInterface(Node):
  """
  Create an BiseNetROS class
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """

    super().__init__('decent_interface')
    
    self.declare_parameter('density_ui_topic', 'density_ui_topic_default')
    self.declare_parameter('map_resol_topic', 'map_resol_topic_default')
    self.declare_parameter('pivot_gps_lu_topic', 'pivot_gps_lu_topic_default')
    self.declare_parameter('pivot_gps_lb_topic', 'pivot_gps_lb_topic_default')
    self.declare_parameter('pivot_gps_ru_topic', 'pivot_gps_ru_topic_default')
    self.declare_parameter('pivot_gps_rb_topic', 'pivot_gps_rb_topic_default')
    self.declare_parameter('mission_cancel_topic', 'mission_cancel_topic_default')
    self.declare_parameter('datum_x_norm_topic', 'datum_x_norm_topic_default')
    self.declare_parameter('datum_y_norm_topic', 'datum_y_norm_topic_default')
    self.declare_parameter('density_save_dir', 'density_save_dir_default')
    self.declare_parameter('map_resol_txt_dir', 'map_resol_txt_dir_default')
    self.declare_parameter('pivot_gps_lu_txt_dir', 'pivot_gps_lu_txt_dir_default')
    self.declare_parameter('pivot_gps_lb_txt_dir', 'pivot_gps_lb_txt_dir_default')
    self.declare_parameter('pivot_gps_ru_txt_dir', 'pivot_gps_ru_txt_dir_default')
    self.declare_parameter('pivot_gps_rb_txt_dir', 'pivot_gps_rb_txt_dir_default')
    self.declare_parameter('datum_txt_dir', 'datum_txt_dir_default')
    self.declare_parameter('trunk_dir', 'trunk_dir_default')
    
    density_ui_topic =self.get_parameter('density_ui_topic').get_parameter_value().string_value
    map_resol_topic =self.get_parameter('map_resol_topic').get_parameter_value().string_value
    pivot_gps_lu_topic =self.get_parameter('pivot_gps_lu_topic').get_parameter_value().string_value
    pivot_gps_lb_topic =self.get_parameter('pivot_gps_lb_topic').get_parameter_value().string_value
    pivot_gps_ru_topic =self.get_parameter('pivot_gps_ru_topic').get_parameter_value().string_value
    pivot_gps_rb_topic =self.get_parameter('pivot_gps_rb_topic').get_parameter_value().string_value
    mission_cancel_topic =self.get_parameter('mission_cancel_topic').get_parameter_value().string_value
    datum_x_norm_topic =self.get_parameter('datum_x_norm_topic').get_parameter_value().string_value
    datum_y_norm_topic =self.get_parameter('datum_y_norm_topic').get_parameter_value().string_value
    
    self.density_save_dir = self.get_parameter('density_save_dir').get_parameter_value().string_value
    head_tail = os.path.split(self.density_save_dir)
    if not os.path.exists(head_tail[0]):
      print("wrong path for density_save_dir, path : ", head_tail[0])
      #rclpy.shutdown()

    self.map_resol_txt_dir = self.get_parameter('map_resol_txt_dir').get_parameter_value().string_value
    head_tail = os.path.split(self.map_resol_txt_dir)
    if not os.path.exists(head_tail[0]):
      print("wrong path for map_resol_txt_dir, path : ", head_tail[0])
      #rclpy.shutdown()

    self.pivot_gps_lu_txt_dir = self.get_parameter('pivot_gps_lu_txt_dir').get_parameter_value().string_value
    head_tail = os.path.split(self.pivot_gps_lu_txt_dir)
    if not os.path.exists(head_tail[0]):
      print("wrong path for pivot_gps_lu_txt_dir, path : ", head_tail[0])
      #rclpy.shutdown()
      
    self.pivot_gps_lb_txt_dir = self.get_parameter('pivot_gps_lb_txt_dir').get_parameter_value().string_value
    head_tail = os.path.split(self.pivot_gps_lb_txt_dir)
    if not os.path.exists(head_tail[0]):
      print("wrong path for pivot_gps_lb_txt_dir, path : ", head_tail[0])
      #rclpy.shutdown()

    self.pivot_gps_ru_txt_dir = self.get_parameter('pivot_gps_ru_txt_dir').get_parameter_value().string_value
    head_tail = os.path.split(self.pivot_gps_ru_txt_dir)
    if not os.path.exists(head_tail[0]):
      print("wrong path for pivot_gps_ru_txt_dir, path : ", head_tail[0])
      #rclpy.shutdown()
      
    self.pivot_gps_rb_txt_dir = self.get_parameter('pivot_gps_rb_txt_dir').get_parameter_value().string_value
    head_tail = os.path.split(self.pivot_gps_rb_txt_dir)
    if not os.path.exists(head_tail[0]):
      print("wrong path for pivot_gps_rb_txt_dir, path : ", head_tail[0])
      #rclpy.shutdown()
    
    self.datum_txt_dir = self.get_parameter('datum_txt_dir').get_parameter_value().string_value
    head_tail = os.path.split(self.datum_txt_dir)
    if not os.path.exists(head_tail[0]):
      print("wrong path for pivot_gps_rb_txt_dir, path : ", head_tail[0])
      #rclpy.shutdown()
    
    trunk_dir =self.get_parameter('trunk_dir').get_parameter_value().string_value
    
    shutil.rmtree(trunk_dir)
    os.mkdir(trunk_dir)
    
    self.subscription_1 = self.create_subscription(Image, density_ui_topic, self.denisty_callback, 3)
    self.subscription_2 = self.create_subscription(Float64, map_resol_topic, self.map_resol_callback, 3)
    self.subscription_3 = self.create_subscription(NavSatFix, pivot_gps_lu_topic, self.gps_lu_callback, 3)
    self.subscription_4 = self.create_subscription(NavSatFix, pivot_gps_lb_topic, self.gps_lb_callback, 3)
    self.subscription_5 = self.create_subscription(NavSatFix, pivot_gps_ru_topic, self.gps_ru_callback, 3)
    self.subscription_6 = self.create_subscription(NavSatFix, pivot_gps_rb_topic, self.gps_rb_callback, 3)
    self.subscription_7 = self.create_subscription(Int8, mission_cancel_topic, self.cancel_callback, 3)
    self.subscription_8 = self.create_subscription(Float64, datum_x_norm_topic, self.datum_x_callback, 3)
    self.subscription_9 = self.create_subscription(Float64, datum_y_norm_topic, self.datum_y_callback, 3)
    
    self.npi_ok_topic_pub = self.create_publisher(Int8, 'decent_voro_start', 1)
    
    timer_period = 0.2 # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    
    self.time_stamp = self.get_clock().now().to_msg()
      
    self.i = 0
    
    self.br = CvBridge()
    self.density_ok = False
    self.map_resol_ok = False
    self.gps_lu_ok = False
    self.gps_lb_ok = False
    self.gps_ru_ok = False
    self.gps_rb_ok = False
    
    self.datum_x_norm = -1.0
    self.datum_y_norm = -1.0
    self.datum_x_norm_new = -1.0
    self.datum_y_norm_new = -1.0
    
  def timer_callback(self):
    
    if self.datum_x_norm_new != self.datum_x_norm or self.datum_y_norm_new != self.datum_y_norm:
      self.datum_x_norm = self.datum_x_norm_new
      self.datum_y_norm = self.datum_y_norm_new
      with open(self.datum_txt_dir, 'w') as f:
        f.writelines(str(self.datum_x_norm) + ' ' + str(self.datum_y_norm))
    
    if self.density_ok is True and self.map_resol_ok is True:
      msg = Int8()
      msg.data = 1
      self.npi_ok_topic_pub.publish(msg)
    
    else:
      msg = Int8()
      msg.data = 0
      self.npi_ok_topic_pub.publish(msg)
    
  def cancel_callback(self, msg):
    self.density_ok = False 
  
  def datum_x_callback(self, msg):
    self.datum_x_norm_new = msg.data 
   
  def datum_y_callback(self, msg):
    self.datum_y_norm_new = msg.data       
  
  def denisty_callback(self, msg):
    rgb_image = self.br.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
    cv2.imwrite(self.density_save_dir, gray_image)
    self.density_ok = True

  def map_resol_callback(self, msg):
    resol = msg.data
    with open(self.map_resol_txt_dir, 'w') as f:
      f.writelines(str(resol))
    self.map_resol_ok = True

  def gps_lu_callback(self, msg):
    lu_lat = msg.latitude
    lu_long = msg.longitude
    with open(self.pivot_gps_lu_txt_dir, 'w') as f:
      f.writelines(str(lu_lat) + ' ' + str(lu_long))
    self.gps_lu_ok = True

  def gps_lb_callback(self, msg):
    lb_lat = msg.latitude
    lb_long = msg.longitude
    with open(self.pivot_gps_lb_txt_dir, 'w') as f:
      f.writelines(str(lb_lat) + ' ' + str(lb_long))
    self.gps_lb_ok = True

  def gps_ru_callback(self, msg):
    ru_lat = msg.latitude
    ru_long = msg.longitude
    with open(self.pivot_gps_ru_txt_dir, 'w') as f:
      f.writelines(str(ru_lat) + ' ' + str(ru_long))
    self.gps_ru_ok = True

  def gps_rb_callback(self, msg):
    rb_lat = msg.latitude
    rb_long = msg.longitude
    with open(self.pivot_gps_rb_txt_dir, 'w') as f:
      f.writelines(str(rb_lat) + ' ' + str(rb_long))
    self.gps_rb_ok = True
    
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  decent_interface_ros = DecentInterface()
  
  # Spin the node so the callback function is called.
  rclpy.spin(decent_interface_ros)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  decent_interface_ros.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
