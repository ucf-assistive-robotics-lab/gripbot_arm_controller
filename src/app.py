#!/usr/bin/env python

import sys
import rospy
from gripbot_core_msgs.srv import SolvePositionIKRequest, SolvePositionIKResponse, SolvePositionIK
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header

def position_kinematic(x, y, z):
    rospy.wait_for_service('/gripbot/kinematics/arm/pos/IKService')
    try:
        ikrequest = SolvePositionIKRequest()
        initial_pose = Pose()
        initial_pose.position.x = x
        initial_pose.position.y = y
        initial_pose.position.z = z

        ikrequest.pose_stamp = [PoseStamped()]
        ikrequest.pose_stamp[0].header = Header(stamp=rospy.Time.now(), frame_id='base_mount_link')

        ikrequest.pose_stamp[0].pose = initial_pose
        seviceHandle = rospy.ServiceProxy('/gripbot/kinematics/arm/pos/IKService', SolvePositionIK)
        ikresponse = seviceHandle(ikrequest)

        print(ikresponse)
    except rospy.ServiceException as e:
        print("Service call failed: %s".format(e))

def usage():
    return "%s [x y]"%sys.argv[0]
  
if __name__ == "__main__":
    rospy.init_node("arm_position")
    if len(sys.argv) == 4:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
        z = int(sys.argv[3])
        position_kinematic(x, y, z)
