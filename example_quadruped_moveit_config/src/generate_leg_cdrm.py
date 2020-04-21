#!/usr/bin/env python

import actionlib
import rospkg
import rospy

from cdrm_msgs.msg import GenerateCdrmAction, GenerateCdrmGoal
from cdrm_msgs.srv import ProcessFile

def main():
    rospy.init_node('example_quadruped_moveit_config_node')

    client = actionlib.SimpleActionClient('/cdrm_server/generate_cdrm', GenerateCdrmAction)
    client.wait_for_server()

    print 'Generating leg CDRM...'

    goal = GenerateCdrmGoal(group_name='al',
                            roadmap_k=10,
                            roadmap_size=5000,
                            resolution=0.025,
                            collide_edges=True,
                            collide_tip_link=False)
    client.send_goal(goal)
    client.wait_for_result()

    print 'Saving CDRM...'

    save_srv = rospy.ServiceProxy('/cdrm_server/save_cdrm', ProcessFile)
    save_srv(rospkg.get_ros_home() + '/example_quadruped.cdrm')

    print 'Done'

if __name__ == '__main__':
    main()
