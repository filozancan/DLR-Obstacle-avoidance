import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from gazebo_msgs.srv import SetEntityState
import xml.etree.ElementTree as ET
import os
from ament_index_python.packages import get_package_share_directory
import random
import math

class Respawner(Node):
    
    def __init__(self):
        super().__init__("respawner")

        self.declare_parameter('package_name', 'oa_drl_control')
        self.declare_parameter('world_file', 'world1.world')
        self.declare_parameter('robot_name', 'burger')

        pkg_name = self.get_parameter('package_name').value
        world_file = self.get_parameter('world_file').value
        self.robot_name = self.get_parameter('robot_name').value

        # Carica gli ostacoli dal file .world
        self.map_obstacles = self.parse_world_file(pkg_name, world_file)
        if self.map_obstacles:
            self.map_min_x = min([wx - (sx / 2.0) for wx, wy, sx, sy in self.map_obstacles])
            self.map_max_x = max([wx + (sx / 2.0) for wx, wy, sx, sy in self.map_obstacles])
            self.map_min_y = min([wy - (sy / 2.0) for wx, wy, sx, sy in self.map_obstacles])
            self.map_max_y = max([wy + (sy / 2.0) for wx, wy, sx, sy in self.map_obstacles])
            
            self.get_logger().info(f'Limiti mappa calcolati: X[{self.map_min_x:.2f}, {self.map_max_x:.2f}], Y[{self.map_min_y:.2f}, {self.map_max_y:.2f}]')
        else:
            self.map_min_x, self.map_max_x = -5.0, 5.0
            self.map_min_y, self.map_max_y = -5.0, 5.0
            self.get_logger().info(f'Fallimento nel calcolo dei limiti, uso default: X[{self.map_min_x:.2f}, {self.map_max_x:.2f}], Y[{self.map_min_y:.2f}, {self.map_max_y:.2f}]')

        self.cb_group = ReentrantCallbackGroup()
        self.set_state_client = self.create_client(SetEntityState, '/set_entity_state', callback_group=self.cb_group)
        self.srv = self.create_service(Trigger, '/randomize_robot_pose', self.handle_randomize_pose, callback_group=self.cb_group)

        self.get_logger().info('Respawner Intelligente V2 (Raycasting) inizializzato.')
    

    def parse_world_file(self, package_name, world_file_name):
        """Legge il file .world e calcola le coordinate esatte di tutti i muri"""
        walls = []
        try:
            pkg_share_dir = get_package_share_directory(package_name)
            world_path = os.path.join(pkg_share_dir, 'worlds', world_file_name) 

            with open(world_path, 'r') as f:
                world_content = f.read()

            clean_content = world_content.replace('ignition::', 'ignition_')
            root = ET.fromstring(clean_content)

            for model in root.findall('.//model'):
                model_name = model.get('name')
                if model_name in ['ground_plane', 'turtlebot3_burger']:
                    continue

                model_pose_tag = model.find('pose')
                mx, my, myaw = 0.0, 0.0, 0.0
                if model_pose_tag is not None:
                    mp_vals = [float(v) for v in model_pose_tag.text.split()]