#!/usr/bin/env python

import math
import rospkg
import rospy

from cdrm_welding_msgs.msg import IntervalWithOptimal
from cdrm_welding_msgs.srv import PlanWeld
from geometry_msgs.msg import Point, Vector3

SERVICE = '/cdrm_welding_node/plan_weld'

def plan_weld():
    rospy.wait_for_service(SERVICE)

    service = rospy.ServiceProxy(SERVICE, PlanWeld)

    weld_points = [Point(x=0.005, y=0, z=0.005), Point(x=0.005, y=-0.2, z=0.005)]
    weld_directions = [Vector3(-0.7071, 0, -0.7071), Vector3(-0.7071, 0, -0.7071)]

    work_range = IntervalWithOptimal(min=math.radians(-15), max=math.radians(15), optimal=0)
    travel_angle = IntervalWithOptimal(min=math.radians(-15), max=math.radians(15), optimal=0)
    ctwd_range=IntervalWithOptimal(min=0, max=0.015, optimal=0)

    try:
        res = service(cdrm_filename=rospkg.get_ros_home() + '/m10ia_on_gantry.welding-cdrm',
                      robot_group_name='robot',
                      robot_positioner_group_name='track',
                      nozzle_link_name='nozzle_link',
                      workpiece_link_name='workpiece_link',
                      planning_timeout=60,
                      weld_points=weld_points,
                      weld_directions=weld_directions,
                      rx_range=work_range,
                      ry_range=travel_angle,
                      ctwd_range=ctwd_range)
    except rospy.ServiceException as e:
        print 'Service exception: ', e

def main():
    plan_weld()

if __name__ == '__main__':
    main()
