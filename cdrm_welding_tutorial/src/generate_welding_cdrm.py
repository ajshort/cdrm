#!/usr/bin/env python

import actionlib
import rospkg
import rospy

from cdrm_welding_msgs.msg import GenerateWeldingCdrmAction, GenerateWeldingCdrmGoal, Interval

def main():
    rospy.init_node('cdrm_welding_tutorial_generate_node')

    client = actionlib.SimpleActionClient('/cdrm_welding_node/generate_welding_cdrm', GenerateWeldingCdrmAction)
    client.wait_for_server()

    print 'Generating welding CDRM...'

    goal = GenerateWeldingCdrmGoal(filename=rospkg.get_ros_home() + '/m10ia_on_gantry.welding-cdrm',
                                   group_name='robot_with_gantry',
                                   end_effector_name='welding_torch',
                                   nozzle_link_name='nozzle_link',
                                   roadmap_size=1000,
                                   roadmap_k=10,
                                   rx=Interval(min=-0.261799, max=0.261799),
                                   ry=Interval(min=-0.610865, max=0.610865),
                                   ctwd=Interval(min=0, max=0.015),
                                   nozzle_resolution=0.001,
                                   robot_resolution=0.01)
    client.send_goal(goal)
    client.wait_for_result()

    print client.get_result()

if __name__ == '__main__':
    main()