import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ur_msgs.srv import SetIO

from std_msgs.msg import Bool
import numpy as np
import time


class MoveRobotNode():
    """MoveRobotNode"""

    def __init__(self):
        rospy.init_node("move_robot_node", anonymous=True)
        rospy.Subscriber('target_pcl', PointCloud2, self.callback)
        rospy.Subscriber('picking_pcl', PointCloud2, self.callback2)
        self.flagMsg = rospy.Publisher('readyToDetct', Bool, queue_size=1)
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"

        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.detectMsg = Bool()
        self.detectMsg.data = True
        self.imworking = True
        self.set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)

        self.apply_vacuum(0)

        self.OFFSET_X = 0.078 # 7.8cm

    def callback2(self, input_ros_msg):
        gen = pc2.read_points(input_ros_msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True)
        int_data = list(gen)[0]

        pointX = int(np.ceil(int_data[0] * 1000)) / 1000
        pointY = int(np.ceil(int_data[1] * 1000)) / 1000
        pointZ = int(np.ceil(int_data[2] * 1000)) / 1000
        print('Robot operating, x : ', pointX, 'y : ', pointY, 'z', pointZ)
        if self.imworking == False :
            self.goConnect(pointX, pointY, pointZ)

    def callback(self, input_ros_msg):
        gen = pc2.read_points(input_ros_msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True)
        int_data = list(gen)[0]

        pointX = int(np.ceil(int_data[0] * 1000)) / 1000
        pointY = int(np.ceil(int_data[1] * 1000)) / 1000
        pointZ = int(np.ceil(int_data[2] * 1000)) / 1000
        print('Robot operating, x : ', pointX, 'y : ', pointY, 'z', pointZ)
        if self.imworking == False :
            self.goPosition(pointX, pointY, pointZ)


    def goHome(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 2.15  # base == shoulder_pan_joint
        joint_goal[1] = -1.96 # shoulder
        joint_goal[2] = 2.16  # elbow
        joint_goal[3] = -1.92
        joint_goal[4] = -1.57
        joint_goal[5] = 0.59

        print('Go Home : ', joint_goal)
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        wpose = move_group.get_current_pose().pose

        self.imworking = False

    def goConnect(self, pointX, pointY, pointZ):

        self.imworking = True
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        pose_goal = geometry_msgs.msg.Pose()

        # [-1, 0, 0, 0] 하는 이유는 바닥면에 수직한 위치
        pose_goal.orientation.x = -1
        pose_goal.orientation.y = 0
        pose_goal.orientation.z = 0
        pose_goal.orientation.w = 0

        pose_goal.position.x = pointX - self.OFFSET_X - 0.05
        pose_goal.position.y = pointY
        pose_goal.position.z = 0.34  # pointZ

        # [-1, 0, 0, 0] 에 대한 trajectory를 구한 뒤 마지막 말단부만 joint변경하기 위해 아래 과정 수행
        self.move_group.set_pose_target(pose_goal)
        moving_plan = self.move_group.plan()
        target_endpoint = np.asarray(moving_plan[1].joint_trajectory.points[-1].positions)
        # 말단 부 Tip angle 계산 후 수정
        target_endpoint[-1] = 0.52

        print('다가가는중 .. ', pose_goal)
        self.move_group.go(target_endpoint, wait=True)
        print(' 포인트 1 도착 ')

        # [-1, 0, 0, 0] 하는 이유는 바닥면에 수직한 위치
        pose_goal.orientation.x = -1
        pose_goal.orientation.y = 0
        pose_goal.orientation.z = 0
        pose_goal.orientation.w = 0

        pose_goal.position.x = pointX - self.OFFSET_X - 0.02
        pose_goal.position.y = pointY
        pose_goal.position.z = 0.34  # pointZ

        # [-1, 0, 0, 0] 에 대한 trajectory를 구한 뒤 마지막 말단부만 joint변경하기 위해 아래 과정 수행
        self.move_group.set_pose_target(pose_goal)
        moving_plan = self.move_group.plan()
        target_endpoint = np.asarray(moving_plan[1].joint_trajectory.points[-1].positions)
        # 말단 부 Tip angle 계산 후 수정
        target_endpoint[-1] = 0.52

        print('다가가는중 .. ', pose_goal)
        self.move_group.go(target_endpoint, wait=True)
        print(' 포인트 1 도착 ')


    def goPosition(self, pointX, pointY, pointZ):

        self.imworking = True
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        pose_goal = geometry_msgs.msg.Pose()

        # [-1, 0, 0, 0] 하는 이유는 바닥면에 수직한 위치
        pose_goal.orientation.x = -1
        pose_goal.orientation.y = 0
        pose_goal.orientation.z = 0
        pose_goal.orientation.w = 0

        pose_goal.position.x = pointX - self.OFFSET_X
        pose_goal.position.y = pointY
        pose_goal.position.z = 0.18#pointZ

        # [-1, 0, 0, 0] 에 대한 trajectory를 구한 뒤 마지막 말단부만 joint변경하기 위해 아래 과정 수행
        self.move_group.set_pose_target(pose_goal)
        moving_plan = self.move_group.plan()
        target_endpoint = np.asarray(moving_plan[1].joint_trajectory.points[-1].positions)
        # 말단 부 Tip angle 계산 후 수정
        target_endpoint[-1] = 0.42

        print('다가가는중 .. ', pose_goal)
        self.move_group.go(target_endpoint, wait=True)
        print(' 포인트 1 도착 ')

        # [-1, 0, 0, 0] 하는 이유는 바닥면에 수직한 위치
        pose_goal.orientation.x = -1
        pose_goal.orientation.y = 0
        pose_goal.orientation.z = 0
        pose_goal.orientation.w = 0

        pose_goal.position.x = pointX - self.OFFSET_X
        pose_goal.position.y = pointY
        pose_goal.position.z = 0.125#pointZ

        # [-1, 0, 0, 0] 에 대한 trajectory를 구한 뒤 마지막 말단부만 joint변경하기 위해 아래 과정 수행
        self.move_group.set_pose_target(pose_goal)
        moving_plan = self.move_group.plan()
        target_endpoint = np.asarray(moving_plan[1].joint_trajectory.points[-1].positions)
        # 말단 부 Tip angle 계산 후 수정
        target_endpoint[-1] = 0.42

        print('다가가는중 .. ', pose_goal)
        self.move_group.go(target_endpoint, wait=True)
        print(' 포인트 1 도착 ')

        self.move_group.stop()
        self.move_group.clear_pose_targets()

        #self.goHome()

    def apply_vacuum(self, OnOff):
        # OnOff = 1 : On
        self.set_io(fun=1, pin=0, state=OnOff)

if __name__ == "__main__":
    robot_control = MoveRobotNode()
    robot_control.goHome()
    #robot_control.goPosition(0.453, 0.071, 0.25)
    rospy.spin()