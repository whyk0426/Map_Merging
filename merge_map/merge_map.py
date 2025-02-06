import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sklearn.cluster import DBSCAN
from collections import defaultdict
import numpy as np
import math
import copy

PI = math.pi

class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')
        self.publisher = self.create_publisher(OccupancyGrid, 'merge_map', 10)  

        self.n = 2
        self.theta = [0] * self.n
        self.xd = [0] * self.n
        self.yd = [0] * self.n

        self.maps_list = [f"map{i}" for i in range(self.n)] 

        for i in range(self.n):
            topic_name = f'/{self.maps_list[i]}'
            setattr(self, f'subscription_{i}', self.create_subscription(
                OccupancyGrid, 
                topic_name, 
                lambda msg, l=i: self.map_callback(msg, l), 10))
            setattr(self, self.maps_list[i], None)


    def figure_out_diff(self, *maps):
        self.map_data = {k: {'r_diff': [], 'theta_diff': [], 'x_diff': [], 'y_diff': []} for k in range(len(maps))}
        for k in range(len(maps)):
            map_copy = self.cluster_map(copy.deepcopy(maps[k]))       
            setattr(self, f'map_copy{k}', map_copy)

            robot_x = - map_copy.info.origin.position.x
            robot_y = - map_copy.info.origin.position.y

            filtered_data = [-1] * len(map_copy.data)

            for y in range(map_copy.info.height):
                for x in range(map_copy.info.width):
                    i = x + y * map_copy.info.width

                    x_diff = round(x * map_copy.info.resolution - robot_x, 2)
                    y_diff = round(y * map_copy.info.resolution - robot_y, 2)
                    theta_diff = round(math.atan2(y_diff, x_diff), 3)
                    r_diff = round(math.sqrt(x_diff**2 + y_diff**2), 3)

                    if ((map_copy.data[i] > 65)):
                        filtered_data[i] = map_copy.data[i]

                        self.map_data[k]['r_diff'].append(r_diff)
                        self.map_data[k]['theta_diff'].append(theta_diff)
                        self.map_data[k]['x_diff'].append(x_diff)
                        self.map_data[k]['y_diff'].append(y_diff)

            self.get_logger().info(f"map_data[{k}]['r_diff']: {self.map_data[k]['r_diff']}, ['theta_diff']: {self.map_data[k]['theta_diff']}")

            map_copy.data = filtered_data
            self.matched_data = {k: {'r_diff': [], 'theta_diff': [], 'x_diff': [], 'y_diff': [], 'base_theta_diff': []} for k in range(len(self.map_data))}

            if (k == 0):
                self.theta[0] = 0
                self.xd[0] = 0
                self.yd[0] = 0
            else:
                best_map_unified = -1
                best_theta, best_xd, best_yd = None, None, None 

                for i, r_parent in enumerate(self.map_data[0]['r_diff']):
                    for compare_k in range(len(self.map_data)): 
                        for j, r_child in enumerate(self.map_data[compare_k]['r_diff']):
                            if (abs(r_parent - r_child) < 0.01):
                                self.matched_data[compare_k]['r_diff'].append(r_child)
                                self.matched_data[compare_k]['theta_diff'].append( - self.map_data[compare_k]['theta_diff'][j])
                                self.matched_data[compare_k]['x_diff'].append(self.map_data[0]['x_diff'][i])
                                self.matched_data[compare_k]['y_diff'].append(self.map_data[0]['y_diff'][i])
                                self.matched_data[compare_k]['base_theta_diff'].append( - self.map_data[0]['theta_diff'][i])

                    for n in range(len(self.matched_data[compare_k]['theta_diff'])):
                        map_parent = self.rotate_map(getattr(self, f'map_copy0'), self.matched_data[compare_k]['base_theta_diff'][n])
                        map_child = self.rotate_map(getattr(self, f'map_copy{compare_k}'), self.matched_data[compare_k]['theta_diff'][n])

                        self.get_logger().info('################################################')
                        self.get_logger().info(f"theta0, theta1: { - self.matched_data[compare_k]['base_theta_diff'][n]}, { - self.matched_data[compare_k]['theta_diff'][n]}")
                        
                        map_unified = self.merge_two_maps(map_parent, map_child)
         
                        if (map_unified > 65):
                            self.get_logger().info(f'!!!!SUCCESS!!!! {map_unified}')

                            if (map_unified > best_map_unified):
                                best_map_unified = map_unified

                                theta_1to0 = self.matched_data[compare_k]['theta_diff'][n] - self.matched_data[compare_k]['base_theta_diff'][n]
                                if (theta_1to0 >= 0):
                                    best_theta = round(theta_1to0 - PI, 2)
                                else:
                                    best_theta = round(theta_1to0 + PI, 2)
                                best_xd = self.matched_data[compare_k]['x_diff'][n]
                                best_yd = self.matched_data[compare_k]['y_diff'][n]
                        
                if (best_map_unified != -1):
                    self.theta[compare_k] = best_theta
                    self.xd[compare_k] = best_xd
                    self.yd[compare_k] = best_yd
                    self.get_logger().info('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
                    self.get_logger().info(f'theta:{self.theta}')
                    self.get_logger().info(f'   xd:{self.xd}')
                    self.get_logger().info(f'   yd:{self.yd}')


    def cluster_map(self, map_data):
        points = []
        width = map_data.info.width
        height = map_data.info.height
        eps = 40 * map_data.info.resolution

        for y in range(height):
            for x in range(width):
                index = y * width + x
                if map_data.data[index] >= 65:
                    points.append((x, y))

        points = np.array(points)

        if len(points) == 0:
            for y in range(height):
                for x in range(width):
                    index = y * width + x
                    map_data.data[index] = -1
            return map_data
        
        dbscan = DBSCAN(eps=eps, min_samples=1)
        labels = dbscan.fit_predict(points)

        cluster_sizes = {}
        for label in set(labels):
            if label == -1:
                continue  
            cluster_sizes[label] = (labels == label).sum()
            self.get_logger().info(f'Cluster {label}: Size = {cluster_sizes[label]}')  

        for label, size in cluster_sizes.items():
            if size >= 5:
                for (x, y), cluster_label in zip(points, labels):
                    if cluster_label == label:
                        index = y * width + x
                        map_data.data[index] = -1

        occupied_indicates = {y * width + x for x, y in points}
        for y in range(height):
            for x in range(width):
                index = y * width + x
                if index not in occupied_indicates:
                    map_data.data[index] = -1
        return map_data
  

    def rotate_map(self, map_data, angle, new_resolution=None):
        robot_x = -map_data.info.origin.position.x
        robot_y = -map_data.info.origin.position.y
        height = map_data.info.height
        width = map_data.info.width
        old_resolution = map_data.info.resolution

        if new_resolution is None:
            new_resolution = 0.5 * old_resolution  

        scale_factor = old_resolution / new_resolution  
        new_height = int(height * scale_factor)  
        new_width = int(width * scale_factor)    

        rotated_map_data = [-1] * (new_height * new_width)

        cos_th = np.cos(-angle)
        sin_th = np.sin(-angle)

        for i in range(new_height):
            for j in range(new_width):
                rotated_x = j * new_resolution
                rotated_y = i * new_resolution

                orig_x = cos_th * (rotated_x - robot_x) - sin_th * (rotated_y - robot_y) + robot_x
                orig_y = sin_th * (rotated_x - robot_x) + cos_th * (rotated_y - robot_y) + robot_y

                orig_j = int(np.floor(orig_x / old_resolution))
                orig_i = int(np.floor(orig_y / old_resolution))

                if 0 <= orig_i < height and 0 <= orig_j < width:
                    rotated_index = i * new_width + j
                    orig_index = orig_i * width + orig_j
                    rotated_map_data[rotated_index] = map_data.data[orig_index]

        rotated_map_msg = OccupancyGrid()
        rotated_map_msg.header = map_data.header
        rotated_map_msg.info = copy.deepcopy(map_data.info)
        rotated_map_msg.info.resolution = new_resolution  
        rotated_map_msg.info.width = new_width
        rotated_map_msg.info.height = new_height
        rotated_map_msg.data = rotated_map_data

        return rotated_map_msg

    # def rotate_map(self, map_data, angle):
    #     robot_x = - map_data.info.origin.position.x
    #     robot_y = - map_data.info.origin.position.y
    #     height = map_data.info.height
    #     width = map_data.info.width
    #     resolution = map_data.info.resolution

    #     rotated_map_data = [-1] * (height * width)

    #     cos_th = np.cos(-angle) 
    #     sin_th = np.sin(-angle)

    #     for i in range(height):
    #         for j in range(width):
    #             rotated_x = j * resolution
    #             rotated_y = i * resolution

    #             orig_x = cos_th * (rotated_x - robot_x) - sin_th * (rotated_y - robot_y) + robot_x
    #             orig_y = sin_th * (rotated_x - robot_x) + cos_th * (rotated_y - robot_y) + robot_y

    #             orig_j = int(np.floor(orig_x / resolution))
    #             orig_i = int(np.floor(orig_y / resolution))

    #             if 0 <= orig_i < height and 0 <= orig_j < width:
    #                 rotated_index = i * width + j
    #                 orig_index = orig_i * width + orig_j
    #                 rotated_map_data[rotated_index] = map_data.data[orig_index]

    #     rotated_map_msg = OccupancyGrid()
    #     rotated_map_msg.header = map_data.header
    #     rotated_map_msg.info = map_data.info
    #     rotated_map_msg.data = rotated_map_data

    #     return rotated_map_msg


    def merge_two_maps(self, map0, map1):
        merged_map = OccupancyGrid()
        merged_map.header = map0.header
        merged_map.header.frame_id = 'map'
        
        min_x = min(map0.info.origin.position.x, map1.info.origin.position.x)
        min_y = min(map0.info.origin.position.y, map1.info.origin.position.y)
        
        max_x = max(map0.info.origin.position.x + (map0.info.width * map0.info.resolution),
                    map1.info.origin.position.x + (map1.info.width * map1.info.resolution))
        
        max_y = max(map0.info.origin.position.y + (map0.info.height * map0.info.resolution),
                    map1.info.origin.position.y + (map1.info.height * map1.info.resolution))
        
        merged_map.info.origin.position.x = min_x
        merged_map.info.origin.position.y = min_y
        merged_map.info.resolution = min(map0.info.resolution, map1.info.resolution)
        merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
        merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))
        
        merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)
        
        size_of_data = 0

        for y in range(map0.info.height):
            for x in range(map0.info.width):
                i = x + y * map0.info.width
                merged_x = int(np.floor((map0.info.origin.position.x + x * map0.info.resolution - min_x) / merged_map.info.resolution))
                merged_y = int(np.floor((map0.info.origin.position.y + y * map0.info.resolution - min_y) / merged_map.info.resolution))
                merged_i = merged_x + merged_y * merged_map.info.width
                merged_map.data[merged_i] = int(0.5 * map0.data[i])
        
        for y in range(map1.info.height):
            for x in range(map1.info.width):
                i = x + y * map1.info.width
                merged_x = int(np.floor((map1.info.origin.position.x + x * map1.info.resolution - min_x) / merged_map.info.resolution))
                merged_y = int(np.floor((map1.info.origin.position.y + y * map1.info.resolution - min_y) / merged_map.info.resolution))
                merged_i = merged_x + merged_y * merged_map.info.width
                if merged_map.data[merged_i] == -1:
                    merged_map.data[merged_i] = int(0.5 * map1.data[i])
                else:
                    merged_map.data[merged_i] = int(merged_map.data[merged_i] + 0.5 * map1.data[i])

                if (merged_map.data[merged_i] > 60):
                    size_of_data = size_of_data + merged_map.data[merged_i]
                    
        return size_of_data
    
    
    def merge_maps(self, *maps):
        merged_map = OccupancyGrid()
        merged_map.header = maps[0].header  
        merged_map.header.frame_id = 'map'

        min_x = min(map.info.origin.position.x + self.xd[i] for i, map in enumerate(maps))
        min_y = min(map.info.origin.position.y + self.yd[i] for i, map in enumerate(maps))

        max_x = max(map.info.origin.position.x + (map.info.width * map.info.resolution) + self.xd[i] for i, map in enumerate(maps))
        max_y = max(map.info.origin.position.y + (map.info.height * map.info.resolution) + self.yd[i] for i, map in enumerate(maps)) 

        merged_map.info.origin.position.x = min_x
        merged_map.info.origin.position.y = min_y
        merged_map.info.resolution = min(map.info.resolution for map in maps)
        merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
        merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))

        merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)
        ratio = [0] * (merged_map.info.width * merged_map.info.height)

        for k in range(len(maps)):
            for y in range(maps[k].info.height):
                for x in range(maps[k].info.width):
                    i = x + y * maps[k].info.width
                    merged_x = int(np.floor((maps[k].info.origin.position.x + self.xd[k] + x * maps[k].info.resolution - min_x) / merged_map.info.resolution))
                    merged_y = int(np.floor((maps[k].info.origin.position.y + self.yd[k] + y * maps[k].info.resolution - min_y) / merged_map.info.resolution))
                    merged_i = merged_x + merged_y * merged_map.info.width
                    if merged_map.data[merged_i] == -1:
                        ratio[merged_i] = 1
                        merged_map.data[merged_i] = maps[k].data[i]
                    else: 
                        ratio[merged_i] += 1  
                        merged_map.data[merged_i] = int(((ratio[merged_i] - 1) / ratio[merged_i]) * merged_map.data[merged_i] 
                                                        + (1 / ratio[merged_i]) * maps[k].data[i])  

        return merged_map


    def map_callback(self, msg, map_id):
        setattr(self, self.maps_list[map_id], msg)
        if all(getattr(self, self.maps_list[j]) is not None for j in range(self.n) if j != map_id):
            maps = [getattr(self, self.maps_list[k]) for k in range(self.n)]
            self.figure_out_diff(*maps)
            rotated_maps = [self.rotate_map(getattr(self, self.maps_list[k]), self.theta[k]) for k in range(self.n)]  
            merged_msg = self.merge_maps(*rotated_maps) 
            self.publisher.publish(merged_msg)  


def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()