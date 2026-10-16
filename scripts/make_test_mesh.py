#!/usr/bin/env python3
# Copyright 2026 CARI-JRL
# SPDX-License-Identifier: Apache-2.0
# Author: Manuel Beschi <manuel.beschi@unibs.it>
"""Synthetic test surface for raster_planner_node, usable without node 2.

Generates a rectangular, slightly curved patch (optionally with a bump, a hole
and a tilt) and either writes it to an ASCII PLY file or publishes it as a
latched shape_msgs/Mesh, like node 2 does.

Examples:
    make_test_mesh.py --output /tmp/test_surface.ply --hole-radius 0.03
    make_test_mesh.py --publish --topic /cloud_merger_node/mesh
"""
import argparse
import math
import sys


def build_mesh(args):
    nx = int(round(args.length / args.resolution)) + 1
    ny = int(round(args.width / args.resolution)) + 1
    tilt = args.tilt
    cx, cy = args.length / 2.0, args.width / 2.0

    def height(x, y):
        z = args.curvature * (x - cx) ** 2  # cylinder-like bend along the long side
        z += args.bump_height * math.exp(-((x - cx) ** 2 + (y - cy) ** 2) / (2 * args.bump_sigma ** 2))
        return z

    vertices = []
    for j in range(ny):
        for i in range(nx):
            x, y = i * args.resolution - cx, j * args.resolution - cy
            z = height(x + cx, y + cy)
            # tilt around the long side (x axis), then lift the patch over the table
            yr = y * math.cos(tilt) - z * math.sin(tilt)
            zr = y * math.sin(tilt) + z * math.cos(tilt)
            vertices.append((x + args.x, yr + args.y, zr + args.z))

    triangles = []
    for j in range(ny - 1):
        for i in range(nx - 1):
            mx = (i + 0.5) * args.resolution - cx
            my = (j + 0.5) * args.resolution - cy
            if args.hole_radius > 0.0 and math.hypot(mx - args.hole_offset, my) < args.hole_radius:
                continue
            a = j * nx + i
            b, c, d = a + 1, a + nx, a + nx + 1
            triangles.append((a, b, c))
            triangles.append((b, d, c))
    return vertices, triangles


def write_ply(path, vertices, triangles):
    with open(path, 'w') as f:
        f.write('ply\nformat ascii 1.0\n')
        f.write(f'element vertex {len(vertices)}\n')
        f.write('property float x\nproperty float y\nproperty float z\n')
        f.write(f'element face {len(triangles)}\n')
        f.write('property list uchar int vertex_indices\nend_header\n')
        for v in vertices:
            f.write(f'{v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n')
        for t in triangles:
            f.write(f'3 {t[0]} {t[1]} {t[2]}\n')


def publish(topic, vertices, triangles):
    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from shape_msgs.msg import Mesh, MeshTriangle

    rclpy.init()
    node = rclpy.create_node('test_mesh_publisher')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(Mesh, topic, qos)
    msg = Mesh()
    msg.vertices = [Point(x=v[0], y=v[1], z=v[2]) for v in vertices]
    msg.triangles = [MeshTriangle(vertex_indices=list(t)) for t in triangles]
    pub.publish(msg)
    node.get_logger().info(
        f'Published {len(vertices)} vertices, {len(triangles)} triangles on {pub.topic_name} '
        '(latched, Ctrl-C to quit)')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', help='PLY file to write')
    p.add_argument('--publish', action='store_true', help='publish as latched shape_msgs/Mesh')
    p.add_argument('--topic', default='mesh', help='topic used with --publish')
    p.add_argument('--length', type=float, default=0.40, help='[m] long side')
    p.add_argument('--width', type=float, default=0.25, help='[m] short side')
    p.add_argument('--resolution', type=float, default=0.005, help='[m] grid step')
    p.add_argument('--curvature', type=float, default=0.5, help='[1/m] bend along the long side')
    p.add_argument('--bump-height', type=float, default=0.03, help='[m]')
    p.add_argument('--bump-sigma', type=float, default=0.04, help='[m]')
    p.add_argument('--hole-radius', type=float, default=0.0, help='[m] 0 = no hole')
    p.add_argument('--hole-offset', type=float, default=0.10, help='[m] hole centre along the long side')
    p.add_argument('--tilt', type=float, default=0.0, help='[rad] tilt around the long side')
    p.add_argument('--x', type=float, default=0.0, help='[m] patch centre')
    p.add_argument('--y', type=float, default=0.0, help='[m] patch centre')
    p.add_argument('--z', type=float, default=0.05, help='[m] patch height')
    args, _ = p.parse_known_args()  # ignore the ROS arguments added by ros2 run

    if not args.output and not args.publish:
        p.error('use --output and/or --publish')
    vertices, triangles = build_mesh(args)
    if args.output:
        write_ply(args.output, vertices, triangles)
        print(f'Wrote {len(vertices)} vertices, {len(triangles)} triangles to {args.output}')
    if args.publish:
        publish(args.topic, vertices, triangles)
    return 0


if __name__ == '__main__':
    sys.exit(main())
