import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import numpy as np

from example_interfaces.msg import Int64

class subscriber_node(Node):
    def __init__(self):
        super().__init__('node1')
        self.get_logger().info("node 1")

        self.pub = self.create_publisher(PointCloud, 'people_points', 10)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.ranges = []
        self.points = []
        self.max_ranges = []

    def scan_callback(self, msg: LaserScan):
        if(self.ranges == []):
            self.ranges = msg
            self.max_ranges = msg.ranges
            return

        self.points = self.find_people_points(msg, self.ranges)

        self.ranges = msg

        temp = PointCloud()
        temp.points = self.points
        self.pub.publish(temp)
        self.points = []
    
    def find_people_points(self, data: LaserScan, prev_data: LaserScan):
        ranges = data.ranges
        prev_range = prev_data.ranges
        people_points = []

        for k in range(len(ranges)):
            diff = abs(ranges[k] - prev_range[k])

            if(ranges[k] < (self.max_ranges[k] - 0.3) and diff > 0.04):
                a = data.angle_increment * k + data.angle_min
                x = math.cos(a) * ranges[k]
                y = math.sin(a) * ranges[k]
                people_points.append((x, y, a))
            else:
                if(ranges[k] == float('inf')):
                    continue

                self.max_ranges[k] += ranges[k]
                self.max_ranges[k] /= 2
        
        for k in range(len(people_points)):
            temp = Point32()
            temp.x = people_points[k][0]
            temp.y = people_points[k][1]
            temp.z = people_points[k][2]
            people_points[k] = temp

        return people_points


    
class publisher_node(Node):
    def __init__(self):
        super().__init__('node2')
        self.get_logger().info("node 2")
        self.points_sub = self.create_subscription(PointCloud, '/people_points', self.distinguish_people, 10)
        self.pub = self.create_publisher(PointCloud, '/person_locations', 10)
        self.people_current = self.create_publisher(Int64, '/people_count_current', 10)
        self.people_total = self.create_publisher(Int64, "/people_count_total", 10)
        self.total_people = 0
        self.index = 0
        self.counter = 0

        self.prev_people_locations = []

    def distinguish_people(self, msg: PointCloud):

        if(len(msg.points) == 0):
            person_publish = PointCloud()
            person_publish.header.frame_id = "laser"
            person_publish.points = []
                
            self.pub.publish(person_publish)
            return 
        
        points = np.array([[p.x, p.y, p.z] for p in msg.points])

        distances = np.sqrt(np.sum((points[:, None, :] - points[None, :, :]) ** 2, axis=-1))

        people = []
        points_left = set(range(len(points)))

        while points_left:
            new_person = [points_left.pop()]

            while True:
                ds = distances[new_person, :]
                good_points = np.where(ds < 0.5)[1]
                new_points = set(good_points) - set(new_person)
                if not new_points:
                    break
                new_person.extend(new_points)
                points_left -= new_points

            people.append(points[new_person])

        people_locations = []
        for k in people:
            if(len(k) > 5):
                person_center = self.find_person_center(k)
                people_locations.append(person_center)

        for j in people_locations:
            close_enough = 0
            majority_vote = 0
            for k in self.prev_people_locations:
                for i in k:
                    diff = math.sqrt( (j.x - i.x)**2 + (j.y - i.y)**2 )
                    if diff < 0.8 or abs(j.z - i.z) < 0.08:
                        close_enough = 1

                majority_vote += len(k)

            if(len(self.prev_people_locations) > 0):
                majority_vote /= len(self.prev_people_locations)
                majority_vote = round(majority_vote)

                if(majority_vote > self.total_people):
                    self.total_people = majority_vote
                    continue
            
            if(close_enough == 0):
                self.total_people += 1

        self.prev_people_locations.append(people_locations)

        if(len(self.prev_people_locations) > 5):
            self.prev_people_locations.pop(0)

        final_people_locations = []
        for k in range(len(people_locations)):
            temp_x = people_locations[k].x
            temp_y = people_locations[k].y
            new_val = Point32()
            new_val.x = temp_x
            new_val.y = temp_y
            new_val.z = 0.0
            final_people_locations.append(new_val)

        person_publish = PointCloud()
        person_publish.header.frame_id = "laser"
        person_publish.points = final_people_locations
        
        self.pub.publish(person_publish)

        people_current = 0
        for k in self.prev_people_locations:
            people_current += len(k)
        
        people_current /= len(self.prev_people_locations)
        people_current = round(people_current)

        current_people = Int64()
        current_people.data = max(people_current, len(people_locations))
        self.people_current.publish(current_people)
        self.get_logger().info("Current People: " + str(current_people))

        total_people = Int64()
        total_people.data = self.total_people
        self.people_total.publish(total_people)
        self.get_logger().info("Total People: " + str(total_people))

    def find_person_center(self, data):
        x = 0
        y = 0
        z = 0
        for k in data:
            x += k[0]
            y += k[1]
            z += k[2]

        x /= len(data)
        y /= len(data)
        z /= len(data)

        ans = Point32()
        ans.x = x
        ans.y = y
        ans.z = z

        return ans


def sub_main(args=None):
    rclpy.init(args=None)
    
    node = subscriber_node()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

def pub_main(args=None):
    rclpy.init(args=None)

    node = publisher_node()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

