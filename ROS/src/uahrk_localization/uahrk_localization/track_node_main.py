#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from TrackNode import TrackNode

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = TrackNode()
    rclpy.spin(minimal_publisher)

if __name__ == "__main__":
    main()
