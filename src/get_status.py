#!/usr/bin/env python3

import rospy
from moveit_commander import MoveGroupCommander
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Pose
from franka_gripper.msg import MoveAction, MoveGoal
import actionlib
import time
import yaml
import rospkg


def control_gripper(iWidth, iSpeed):
    client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    client.wait_for_server()
    goal = MoveGoal(width = iWidth, speed = iSpeed)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

def close_gripper():
    control_gripper(0, 0.04)

def open_gripper():
    control_gripper(0.08,0.04)

def test_closing_gripper():
    open_gripper()
    for i in range(9):
        control_gripper((0.08-i*0.01),0.04)
        time.sleep(2)

def test_opening_gripper():
    close_gripper()
    for i in range(9):
        control_gripper(i*0.01, 0.04)
        time.sleep(2)

def cartesian_go_to(iCommander,iPose):
    iCommander.set_pose_target(iPose)
    iCommander.go(wait=True)
    iCommander.stop()

def joint_go_to(iCommander, iJointArray):
    iCommander.go(iJointArray, wait=True)
    iCommander.stop()

def load_yaml(iPath):
    with open(iPath,"r") as stream:
        oData=yaml.safe_load(stream)
    return oData

def dict_to_pose(iPoseDict):
    oPose=Pose()
    oPose.position.x=iPoseDict['position']['x']
    oPose.position.y=iPoseDict['position']['y']
    oPose.position.z=iPoseDict['position']['z']
    oPose.orientation.x=iPoseDict['orientation']['x']
    oPose.orientation.y=iPoseDict['orientation']['y']
    oPose.orientation.z=iPoseDict['orientation']['z']
    oPose.orientation.w=iPoseDict['orientation']['w']
    return oPose

def load_pose():  
    rospack=rospkg.RosPack()
    file_path=rospack.get_path("panda_command")+"/config/poseList.yaml"
    lData=load_yaml(file_path)
    
    oPrePickingPose=dict_to_pose(lData["prepicking_pose"])
    oPickingPose=dict_to_pose(lData["picking_pose"])
    oPrePlacePose=dict_to_pose(lData["preplace_pose"])
    oPlacePose=dict_to_pose(lData["place_pose"])
        
    return oPrePickingPose, oPickingPose, oPrePlacePose,oPlacePose 



def demo_pick_and_place():
    mPrepickingPose, mPickingPose, mPrePlacePose, mPlacePose=load_pose()
    print(mPrepickingPose)
    print(mPickingPose)
    print(mPrePlacePose)
    print(mPlacePose)

if __name__ == '__main__':
    rospy.init_node('get_joint_status')
    rospy.wait_for_message('move_group/status', GoalStatusArray)
    commander = MoveGroupCommander('panda_arm')
    joint=commander.get_current_joint_values()
    ee_pose=commander.get_current_pose().pose
    
    pose_goal=Pose()

    pose_goal.orientation=ee_pose.orientation
    pose_goal.position.x=0.10
    pose_goal.position.y=0.50
    pose_goal.position.z=0.600
    print(pose_goal)
    # cartesian_go_to(commander, pose_goal)
    # open_gripper()
    # commander.set_named_target('ready')
    # commander.go()
    demo_pick_and_place()