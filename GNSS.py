
#!/usr/bin/env python

"""gps_goal_node.py

This file is home to the gps_goal node. The node only offer services.

This file contains two services: one for converting a relative position,
GNSS position, and GNSS goal position into a relative goal in meters,
another service for converting a relative goal, relative position, and
GNSS position into a GNSS goal position to be consumed by the move base
package.
"""

import rospy
import math
import unittest
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Point
from geographiclib.geodesic import Geodesic
from gps_goal.srv import GetGoalPosition, GetGoalPositionRequest
from gps_goal.srv import GetGoalToGPS, GetGoalToGPSRequest


class GpsGoal():
    """GpsGoal contains functionality pertaining to the gps_goal node."""

    # The name of the node must remain constant.
    NODE_NAME = 'gps_goal'

    @staticmethod
    def gps_to_manhatten_distance(origin: tuple[float, float],
                                  goal: tuple[float, float]
                                  ) -> tuple[float, float]:
        """Gets the manhattan distance in meters between two latitude and
        longitude coordinates.

        :param origin: Latitude and longitude of a current position.
        :param goal: Latitude and longitude of the goal position.
        :return: A tuple containing the manhattan distance between
        origin and goal in meters.
        """
        # A Geodesic is the shortest path between two points on a curved
        # surface. We calculate the distance and azimuth between two
        # GPS points.
        g = Geodesic.WGS84.Inverse(origin[0], origin[1], goal[0], goal[1])
# g is a Geodesic dictionary:
        # https://geographiclib.sourceforge.io/html/python/interface.html#dict
        distance = g['s12']  # In meters.
        azimuth = g['azi1']  # Azimuth of line at point 1 in degrees.

        rospy.logdebug(
            'Distance and azimuth from {0} to {1} is {2} and {3}, respectively'
            .format(origin, goal, distance, azimuth))

        # Convert distance and azimuth into x, y translation in meters by
        # finding the lengths of a right triangle.
        azimuth = math.radians(azimuth)

        # CAH = Cosine, Adjacent, Hypotenuse.
        x = math.cos(azimuth) * distance

        # SOH = Sin, Oppostie, Hypotenuse.
        y = math.sin(azimuth) * distance

        rospy.logdebug('Manhatten distance from {0} to {1} is {2}, {3} meters'
                       .format(origin, goal, x, y))

        return x, y

    @staticmethod
    def relative_goal_to_gps(origin: tuple[float, float],
                             origin_position: Point,
                             goal_position: Point
                             ) -> tuple[float, float]:
        """Converts a goal in meters, relative to a known origin, into a
        latitude and longitude coordinate.

        :param origin: Latitude and longitude of the known origin.
        :param origin_position: Relative origin position in meters.
        :param goal_position: Goal position in meters relative to the origin.
        :return: Latitude and longitude coordinate of the goal.
        """
        # Get the difference between the two positions in meters.
        offset_x = goal_position.x - origin_position.x
        offset_y = goal_position.y