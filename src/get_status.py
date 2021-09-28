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

def place_and_pick(iCommander):
    mPrepickingPose, mPickingPose, mPrePlacePose, mPlacePose=load_pose()

    #step 0 open gripper
    open_gripper()
    # time.sleep(1)

    #step one go to start position
    iCommander.set_named_target('ready')
    iCommander.go()
    # time.sleep(2)

    #step two go to prepicking place
    cartesian_go_to(iCommander, mPrePlacePose)
    # time.sleep(2)
    
    #step three go to picking place
    cartesian_go_to(iCommander, mPlacePose)
    # time.sleep(1)
    
    #step four pick the cube
    control_gripper(0.023,0.04)
    # time.sleep(1)

    #step five go to prepicking place
    cartesian_go_to(iCommander, mPrePlacePose)
    
    #step six go to start position
    iCommander.set_named_target('ready')
    iCommander.go()
    # time.sleep(2)
    
    #step seven go to preplace place
    cartesian_go_to(iCommander,mPrepickingPose)
    # time.sleep(1)

    #step eight go to place place
    cartesian_go_to(iCommander, mPickingPose)
    # time.sleep(2)
    
    #step nine drop the cube
    open_gripper()
    # time.sleep(1)

    #step ten go to preplace place
    cartesian_go_to(iCommander,mPrepickingPose)
    # time.sleep(2)

    #step eleven go to start position
    iCommander.set_named_target('ready')
    iCommander.go()

def pick_and_place(iCommander):
    mPrepickingPose, mPickingPose, mPrePlacePose, mPlacePose=load_pose()

    #step 0 open gripper
    open_gripper()
    # time.sleep(1)

    #step one go to start position
    iCommander.set_named_target('ready')
    iCommander.go()
    # time.sleep(2)

    #step two go to prepicking place
    cartesian_go_to(iCommander, mPrepickingPose)
    # time.sleep(2)
    
    #step three go to picking place
    cartesian_go_to(iCommander, mPickingPose)
    # time.sleep(1)
    
    #step four pick the cube
    control_gripper(0.023,0.04)
    # time.sleep(1)

    #step five go to prepicking place
    cartesian_go_to(iCommander, mPrepickingPose)
    
    #step six go to start position
    iCommander.set_named_target('ready')
    iCommander.go()
    # time.sleep(2)
    
    #step seven go to preplace place
    cartesian_go_to(iCommander,mPrePlacePose)
    # time.sleep(1)

    #step eight go to place place
    cartesian_go_to(iCommander, mPlacePose)
    # time.sleep(2)
    
    #step nine drop the cube
    open_gripper()
    # time.sleep(1)

    #step ten go to preplace place
    cartesian_go_to(iCommander,mPrePlacePose)
    # time.sleep(2)

    #step eleven go to start position
    iCommander.set_named_target('ready')
    iCommander.go()




def demo_pick_and_place_loop(iCommander):
    for i in range(10):
        pick_and_place(iCommander)
        place_and_pick(iCommander)






if __name__ == '__main__':
    rospy.init_node('get_joint_status')
    rospy.wait_for_message('move_group/status', GoalStatusArray)
    commander = MoveGroupCommander('panda_arm')
    joint=commander.get_current_joint_values()
    ee_pose=commander.get_current_pose().pose
    # pick_and_place(commander)
    # place_and_pick(commander)
    demo_pick_and_place_loop(commander)