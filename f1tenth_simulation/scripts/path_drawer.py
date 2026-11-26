#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_srvs.srv import Empty


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def catmull_rom(p0, p1, p2, p3, t):
    """
    Catmull-Rom spline (uniform).
    p0..p3 are 2D points (x,y). t in [0,1].
    Returns (x,y).
    """
    t2 = t * t
    t3 = t2 * t
    x0, y0 = p0
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3

    # 0.5 * ( 2P1 + (-P0+P2)t + (2P0-5P1+4P2-P3)t^2 + (-P0+3P1-3P2+P3)t^3 )
    x = 0.5 * (2*x1 + (-x0 + x2)*t + (2*x0 - 5*x1 + 4*x2 - x3)*t2 + (-x0 + 3*x1 - 3*x2 + x3)*t3)
    y = 0.5 * (2*y1 + (-y0 + y2)*t + (2*y0 - 5*y1 + 4*y2 - y3)*t2 + (-y0 + 3*y1 - 3*y2 + y3)*t3)
    return (x, y)


class PathDrawer(Node):
    """
    Draw a Path in RViz by clicking points (Publish Point tool).

    Publishes:
      /drawn_plan_raw  (nav_msgs/Path)  raw points (frame odom)
      /drawn_plan      (nav_msgs/Path)  smoothed spline (frame odom)  <-- MPC uses this

    Subscribes:
      /clicked_point (geometry_msgs/PointStamped)
      /odom          (nav_msgs/Odometry)

    Service:
      /clear_drawn_plan (std_srvs/Empty)
    """

    def __init__(self):
        super().__init__("path_drawer")

        self.frame_id = "/r1/odom"

        # Spline parameters (tweakable)
        self.points_per_segment = 15   # more = smoother, heavier
        self.min_points_for_spline = 4 # need at least 4 to really spline nicely

        self.have_odom = False
        self.started = False

        # Store raw xy points (including anchor pose)
        self.raw_xy = []

        # ROS
        self.sub_click = self.create_subscription(PointStamped, "/clicked_point", self.click_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, "/r1/odom", self.odom_callback, 20)

        self.pub_raw = self.create_publisher(Path, "/drawn_plan_raw", 10)
        self.pub_smooth = self.create_publisher(Path, "/drawn_plan", 10)

        self.srv_clear = self.create_service(Empty, "/clear_drawn_plan", self.clear_callback)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_q = None  # keep latest orientation if you ever want it

        self.get_logger().info(
            "PathDrawer (with splines) ready.\n"
            "- RViz: use 'Publish Point' to add points\n"
            "- /drawn_plan_raw: raw clicked points\n"
            "- /drawn_plan: smoothed spline (what MPC should follow)\n"
            "- Clear: ros2 service call /clear_drawn_plan std_srvs/srv/Empty {}"
        )

    def odom_callback(self, msg: Odometry):
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        self.robot_q = msg.pose.pose.orientation
        self.have_odom = True

    def clear_callback(self, request, response):
        self.raw_xy = []
        self.started = False
        # publish empty paths so RViz clears
        self.pub_raw.publish(self.make_path_msg([]))
        self.pub_smooth.publish(self.make_path_msg([]))
        self.get_logger().warn("Cleared drawn plans.")
        return response

    def click_callback(self, msg: PointStamped):
        if not self.have_odom:
            self.get_logger().warn("Ignoring click: no /odom yet.")
            return

        # 1) Anchor at current robot pose (first time only)
        if not self.started:
            self.raw_xy.append((self.robot_x, self.robot_y))
            self.started = True
            self.get_logger().info(f"Anchored at robot pose ({self.robot_x:.2f}, {self.robot_y:.2f}).")

        # 2) Append clicked point
        self.raw_xy.append((float(msg.point.x), float(msg.point.y)))

        # Publish raw
        self.pub_raw.publish(self.make_path_msg(self.raw_xy))

        # Publish smooth
        smooth_xy = self.smooth_path(self.raw_xy)
        self.pub_smooth.publish(self.make_path_msg(smooth_xy))

        self.get_logger().info(f"Added point ({msg.point.x:.2f}, {msg.point.y:.2f}). Total raw: {len(self.raw_xy)}")

    def make_path_msg(self, xy_list):
        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = now

        for (x, y) in xy_list:
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = now
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        return path

    def smooth_path(self, raw_xy):
        """
        Returns a smoothed list of (x,y).
        If not enough points, returns raw.
        """
        n = len(raw_xy)
        if n < 2:
            return raw_xy

        # With 2-3 points, just densify linearly (still nice for MPC)
        if n < self.min_points_for_spline:
            return self.linear_densify(raw_xy, per_seg=self.points_per_segment)

        pts = raw_xy
        out = []

        # For endpoints, we "extend" by repeating end points (common trick)
        for i in range(n - 1):
            # segment goes from p1 to p2, using p0,p1,p2,p3 context
            p1 = pts[i]
            p2 = pts[i + 1]
            p0 = pts[i - 1] if i - 1 >= 0 else pts[i]
            p3 = pts[i + 2] if i + 2 < n else pts[i + 1]

            # sample this segment
            for s in range(self.points_per_segment):
                t = s / float(self.points_per_segment)
                out.append(catmull_rom(p0, p1, p2, p3, t))

        # ensure the final raw point is included
        out.append(pts[-1])

        # optional: remove points that are too close (keeps /drawn_plan lighter)
        out = self.prune_close(out, min_dist=0.02)
        return out

    def linear_densify(self, xy, per_seg=10):
        out = []
        for i in range(len(xy) - 1):
            x0, y0 = xy[i]
            x1, y1 = xy[i + 1]
            for s in range(per_seg):
                a = s / float(per_seg)
                out.append((x0*(1-a) + x1*a, y0*(1-a) + y1*a))
        out.append(xy[-1])
        return out

    def prune_close(self, xy, min_dist=0.02):
        if not xy:
            return xy
        out = [xy[0]]
        for p in xy[1:]:
            x0, y0 = out[-1]
            x1, y1 = p
            if (x1-x0)*(x1-x0) + (y1-y0)*(y1-y0) >= min_dist*min_dist:
                out.append(p)
        return out


def main():
    rclpy.init()
    node = PathDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()