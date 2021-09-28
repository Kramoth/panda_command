import yaml
import rospkg
import rospy
from geometry_msgs.msg import Pose

def dictToPose(iPoseDict):
	oPose=Pose()
	oPose.position.x=iPoseDict['position']['x']
	oPose.position.y=iPoseDict['position']['y']
	oPose.position.z=iPoseDict['position']['z']
	oPose.orientation.x=iPoseDict['orientation']['x']
	oPose.orientation.y=iPoseDict['orientation']['y']
	oPose.orientation.z=iPoseDict['orientation']['z']
	oPose.orientation.w=iPoseDict['orientation']['w']
	return oPose

if __name__ == '__main__':
	rospack=rospkg.RosPack()
	file_path=rospack.get_path("panda_command")+"/config/poseList.yaml"
	with open(file_path,"r") as stream:
		data_loaded=yaml.safe_load(stream)

	# print(data_loaded)
	print(data_loaded["prepicking_pose"])
	mPose=dictToPose(data_loaded["prepicking_pose"])
	print(mPose)