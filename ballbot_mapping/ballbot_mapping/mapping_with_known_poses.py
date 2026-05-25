#!/usr/bin/env python3
import rclpy
from rclpy import node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, LookupException
import math
from tf_transformations import euler_from_quaternion


PRIOR_PROB = 0.5
OCC_PROB = 0.9
FREE_PROB = 0.35
class Pose:
    def __init__(self, x, y, theta = 0):
        self.x = x
        self.y = y

def coordinates_to_pose(px, py, map_info: MapMetaData):
    pose = Pose(0, 0, 0)
    pose.x = int(round((px - map_info.origin.position.x) / map_info.resolution))
    pose.y = int(round((py - map_info.origin.position.y) / map_info.resolution))
    return pose

def poseOnMap(pose: Pose, map_info: MapMetaData):
    return pose.x < map_info.width and pose.x >= 0 and pose.y < map_info.height and pose.y >= 0

def poseToCell(pose: Pose, map_info: MapMetaData):
    return int(map_info.width * pose.y + pose.x)


def bresenham(start: Pose, end: Pose):
    line = []

    dx = end.x - start.x
    dy = end.y - start.y

    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx = xsign
        xy = 0
        yx = 0
        yy = ysign
    else:
        tmp = dx
        dx = dy
        dy = tmp
        xx = 0
        xy = ysign
        yx = xsign
        yy = 0

    D = 2 * dy - dx
    y = 0

    for i in range(dx + 1):
        line.append(Pose(start.x + i * xx + y * yx, start.y + i * xy + y * yy))
        if D >= 0:
            y += 1
            D -= 2 * dx
        D += 2 * dy

    return line


def inverseSensorModel(p_robot: Pose, p_beam: Pose):
    occ_values = []
    line = bresenham(p_robot, p_beam)

    for p in line[:-1]:
        occ_values.append((FREE_PROB))

    occ_values.append((line[-1], OCC_PROB))
    return occ_values

def prob2logodds(p):
    return math.log(p / (1 - p))

def logodds2prob(logodds):
    try:
        return 1 / (1 + math.exp(-logodds))
    except OverflowError:
        return 1 if logodds>0 else 0.0




class Mapping(node.Node):
    def __init__(self, name: str):
        super().__init__(name)
        self.declare_parameter("width", 50)
        self.declare_parameter("height", 50)
        self.declare_parameter("resolution", 0.1)

        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value
        self.resolution = self.get_parameter("resolution").value

        self.map_ = OccupancyGrid()

        self.map_.info.resolution = self.resolution
        self.map_.info.width = round(self.width / self.resolution)
        self.map_.info.height = round(self.height / self.resolution)
        self.map_.info.origin.position.x = float(round(-self.width / 2))
        self.map_.info.origin.position.y = -self.height / 2
        self.map_.header.frame_id = "odom"

        self.map_.data = [-1] * (self.map_.info.width * self.map_.info.height)

        self.probability_map = [prob2logodds(PRIOR_PROB)] * (self.map_.info.width * self.map_.info.height)

        self.pub_ = self.create_publisher(OccupancyGrid, "map", 10)
        self.sub_ = self.create_subscription(LaserScan, "scan", self.scan_callback, 10)
        self.timer_ = self.create_timer(1.0, self.timer_call_back)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Mapping initialized")

    def scan_callback(self, scan: LaserScan):
        try:
            t = self.tf_buffer.lookup_transform(self.map_.header.frame_id, scan.header.frame_id, rclpy.time.Time())
        except LookupException:
            self.get_logger().info("Waiting for transform")
            return

        robot_p = coordinates_to_pose(t.transform.translation.x, t.transform.translation.y, self.map_.info)

        if not poseOnMap(robot_p, self.map_.info):
            self.get_logger().info("Robot outside of map")
            return

        roll, pitch, yaw = euler_from_quaternion((t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w))

        for i in range(len(scan.ranges)):
            if math.isinf(scan.ranges[i]):
                continue
            angle = scan.angle_min + (i * scan.angle_increment) + yaw
            px = scan.ranges[i] * math.cos(angle)
            py = scan.ranges[i] * math.sin(angle)
            px += t.transform.translation.x
            py += t.transform.translation.y
            beam_p = coordinates_to_pose(px, py, self.map_.info)
            if not poseOnMap(beam_p, self.map_.info):
                continue

            poses = inverseSensorModel(robot_p, beam_p)
            for p, value in poses:
                beam_cell = poseToCell(p, self.map_.info)
                self.probability_map[beam_cell] += prob2logodds(value) - prob2logodds(PRIOR_PROB)

        # robot_cell = poseToCell(robot_p, self.map_.info) prints the cell the robot is in
        # self.map_.data[robot_cell] = 100

        self.pub_.publish(self.map_)


    def timer_call_back(self):
        self.map_.header.stamp = self.get_clock().now().to_msg()
        self.map_.data = [int(logodds2prob(p)) for p in self.probability_map]
        self.pub_.publish(self.map_)

def main():
    rclpy.init()
    node = Mapping("mapping")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
