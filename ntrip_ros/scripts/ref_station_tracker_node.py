#!/usr/bin/env python3

import rclpy
from reference_station_tracker.ReferenceStationTracker import ReferenceStationTracker


def main(args=None):
    rclpy.init(args=None)
    node = ReferenceStationTracker()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
