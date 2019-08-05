#!/usr/bin/env python

import sys
import rospy
from gripbot_core_msgs.srv import SolvePositionIKRequest, SolvePositionIKResponse, SolvePositionIK
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
from std_msgs.msg import String
from std_msgs.msg import Float64

joint_state_publishers = {}


def position_kinematic(x, y, z):
    rospy.wait_for_service('/gripbot/kinematics/arm/IKService')
    try:
        ikrequest = SolvePositionIKRequest()
        initial_pose = Pose()
        initial_pose.position.x = x
        initial_pose.position.y = y
        initial_pose.position.z = z

        ikrequest.pose_stamp = [PoseStamped()]
        ikrequest.pose_stamp[0].header = Header(
            stamp=rospy.Time.now(), frame_id='base_mount_link')

        ikrequest.pose_stamp[0].pose = initial_pose
        seviceHandle = rospy.ServiceProxy(
            '/gripbot/kinematics/arm/IKService', SolvePositionIK)
        ikresponse = seviceHandle(ikrequest)
    except Exception as e:
        print("Application Failed because of something")
        exit()

    if ikresponse.isValid[0] == True:
        return ikresponse.joints
    return None


def publishJoint(jointName, position):
    tname = jointName.split('_')
    tname[len(tname)-1] = "controller"
    jointName = "_".join(tname)

    if jointName not in joint_state_publishers.keys():
        topic = 'gripbot/{}/command'.format(jointName)
        pub = rospy.Publisher(topic, Float64, queue_size=10)
        joint_state_publishers[jointName] = pub

    joint_state_publishers[jointName].publish(position)


if __name__ == "__main__":
    rospy.init_node("arm1_position")
    if len(sys.argv) == 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        joint_state = position_kinematic(x, y, z)

        for i in range(len(joint_state[0].name)):
            jname = joint_state[0].name[i]
            jpos = joint_state[0].position[i]

            publishJoint(jname, jpos)
