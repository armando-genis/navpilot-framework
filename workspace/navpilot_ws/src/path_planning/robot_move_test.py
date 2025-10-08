#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

def euler_to_quaternion(roll, pitch, yaw):
    # Standard REP-103 RPY -> quaternion
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    qw = cr*cp*cy + sr*sp*sy
    return (qx, qy, qz, qw)

class MovingTF(Node):
    def __init__(self):
        super().__init__('moving_transform_publisher')

        # Parameters (radians; identity orientation by default)
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame',  'velodyne')
        self.declare_parameter('start_xyz',    [0.0, 0.0, 2.1])       # x,y,z
        self.declare_parameter('start_rpy',    [0.0, 0.0, 0.0])       # roll,pitch,yaw (rad)
        self.declare_parameter('use_identity_orientation', True)      # force same rotation as parent
        self.declare_parameter('axis',         'x')                   # 'x'|'y'|'z'
        self.declare_parameter('distance',     90.0)                   # meters
        self.declare_parameter('duration',     50.0)                  # seconds
        self.declare_parameter('rate',         50.0)                  # Hz
        self.declare_parameter('hold_final',   True)                  # keep publishing last pose

        gp = lambda n: self.get_parameter(n).get_parameter_value()
        self.parent_frame = gp('parent_frame').string_value
        self.child_frame  = gp('child_frame').string_value
        self.start_xyz    = [float(v) for v in gp('start_xyz').double_array_value]
        self.start_rpy    = [float(v) for v in gp('start_rpy').double_array_value]
        self.use_identity = gp('use_identity_orientation').bool_value
        self.axis         = gp('axis').string_value.lower()
        self.distance     = float(gp('distance').double_value)
        self.duration     = float(gp('duration').double_value)
        self.rate         = float(gp('rate').double_value)
        self.hold_final   = gp('hold_final').bool_value

        assert self.axis in ('x','y','z'), "axis must be one of ['x','y','z']"
        self.axis_idx = {'x':0,'y':1,'z':2}[self.axis]

        # Orientation: identity if you want child aligned with parent (same rotation as map)
        if self.use_identity:
            self.q = (0.0, 0.0, 0.0, 1.0)
        else:
            self.q = euler_to_quaternion(*self.start_rpy)

        self.br = TransformBroadcaster(self)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0/self.rate, self._tick)
        self.done = False

        self.get_logger().info(
            f"{self.parent_frame}->{self.child_frame} moving {self.axis}+={self.distance} m "
            f"over {self.duration}s @ {self.rate} Hz | "
            f"orientation={'IDENTITY' if self.use_identity else 'RPY'}"
        )

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self.start_time).nanoseconds * 1e-9
        progress = 1.0 if self.duration <= 0.0 else min(1.0, max(0.0, dt / self.duration))
        offset = self.distance * progress

        # Translation along parent's chosen axis
        x, y, z = self.start_xyz
        if   self.axis_idx == 0: x += offset
        elif self.axis_idx == 1: y += offset
        else:                    z += offset

        # Publish TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id  = self.child_frame
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x, t.transform.rotation.y, \
        t.transform.rotation.z, t.transform.rotation.w = self.q
        self.br.sendTransform(t)

        if progress >= 1.0 and not self.hold_final and not self.done:
            self.get_logger().info("Motion complete; stopping publisher.")
            self.done = True
            self.timer.cancel()

def main():
    rclpy.init()
    node = MovingTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
