import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ur_msgs.srv import SetIO
from std_msgs.msg import Bool
import numpy as np


class MoveRobotNode():
    def __init__(self):
        rospy.init_node("MoveRobotNode", anonymous=True)
        rospy.Subscriber('TargetPCL', PointCloud2, self.ConnectorCallback)
        rospy.Subscriber('PickingPCL', PointCloud2, self.PikcingTargetCallback)
        self.FLAGMSG = rospy.Publisher('ReadyToDetect', Bool, queue_size=1)
        moveit_commander.roscpp_initialize(sys.argv)
        GROUPNAME = "manipulator"

        self.move_group = moveit_commander.MoveGroupCommander(GROUPNAME)

        self.DETECTMSG = Bool()
        self.DETECTMSG.data = True
        self.IMWORKING = True # True로 시작해서 -> GoHome 이동 후 False로 명령 대기
        self.SetIO = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)

        self.applyVaccum(0)

        # Constant
        self.OFFSET_X = 0.078 # 7.8cm
        self.TOOL_ORIENTATION = 0.52 # 말단부 Z축 회전 정도(Radian)
        self.CONNECT_TARGET_HEIGHT = 0.34
        self.PICKING_TARGET_HEIGHT_1 = 0.18
        self.PICKING_TARGET_HEIGHT_2 = 0.125

    def ConnectorCallback(self, input_ros_msg):
        gen = pc2.read_points(input_ros_msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True)
        int_data = list(gen)[0]

        POINT_X = int(np.ceil(int_data[0] * 1000)) / 1000
        POINT_Y = int(np.ceil(int_data[1] * 1000)) / 1000
        POINT_Z = int(np.ceil(int_data[2] * 1000)) / 1000
        print('Robot operating, x : ', POINT_X, 'y : ', POINT_Y, 'z', POINT_Z)
        if self.IMWORKING == False :
            self.goConnect(POINT_X, POINT_Y, POINT_Z)

    def PikcingTargetCallback(self, input_ros_msg):
        gen = pc2.read_points(input_ros_msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True)
        int_data = list(gen)[0]

        pointX = int(np.ceil(int_data[0] * 1000)) / 1000
        pointY = int(np.ceil(int_data[1] * 1000)) / 1000
        pointZ = int(np.ceil(int_data[2] * 1000)) / 1000
        print('Robot operating, x : ', pointX, 'y : ', pointY, 'z', pointZ)
        if self.IMWORKING == False :
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
        #wpose = move_group.get_current_pose().pose

        self.IMWORKING = False

    def goConnect(self, POINT_X, POINT_Y, POINT_Z):

        self.IMWORKING = True
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        TargetPose = geometry_msgs.msg.Pose()


        # [-1, 0, 0, 0] 하는 이유는 바닥면에 수직한 위치
        # Target에 대한 1차 접
        TargetPose.orientation.x = -1
        TargetPose.orientation.y = 0
        TargetPose.orientation.z = 0
        TargetPose.orientation.w = 0

        TargetPose.position.x = POINT_X - self.OFFSET_X - 0.05
        TargetPose.position.y = POINT_Y
        TargetPose.position.z = self.CONNECT_TARGET_HEIGHT  # pointZ

        # [-1, 0, 0, 0] 에 대한 trajectory를 구한 뒤 마지막 말단부만 joint변경하기 위해 아래 과정 수행
        self.move_group.set_pose_target(TargetPose)
        moving_plan = self.move_group.plan()
        target_endpoint = np.asarray(moving_plan[1].joint_trajectory.points[-1].positions)
        # 말단 부 Tip angle 계산 후 수정
        target_endpoint[-1] = self.TOOL_ORIENTATION
        self.move_group.go(target_endpoint, wait=True)


        # [-1, 0, 0, 0] 하는 이유는 바닥면에 수직한 위치
        # Target에 대한 1차 접근
        TargetPose.orientation.x = -1
        TargetPose.orientation.y = 0
        TargetPose.orientation.z = 0
        TargetPose.orientation.w = 0

        TargetPose.position.x = POINT_X - self.OFFSET_X - 0.02
        TargetPose.position.y = POINT_Y
        TargetPose.position.z = self.CONNECT_TARGET_HEIGHT  # pointZ

        # [-1, 0, 0, 0] 에 대한 trajectory를 구한 뒤 마지막 말단부만 joint변경하기 위해 아래 과정 수행
        self.move_group.set_pose_target(TargetPose)
        moving_plan = self.move_group.plan()
        target_endpoint = np.asarray(moving_plan[1].joint_trajectory.points[-1].positions)
        # 말단 부 Tip angle 계산 후 수정
        target_endpoint[-1] = self.TOOL_ORIENTATION
        self.move_group.go(target_endpoint, wait=True)


    def goPosition(self, pointX, pointY, pointZ):

        self.IMWORKING = True
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
        pose_goal.position.z = self.PICKING_TARGET_HEIGHT_1

        # [-1, 0, 0, 0] 에 대한 trajectory를 구한 뒤 마지막 말단부만 joint변경하기 위해 아래 과정 수행
        self.move_group.set_pose_target(pose_goal)
        moving_plan = self.move_group.plan()
        target_endpoint = np.asarray(moving_plan[1].joint_trajectory.points[-1].positions)
        # 말단 부 Tip angle 계산 후 수정
        target_endpoint[-1] = self.TOOL_ORIENTATION

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
        pose_goal.position.z = self.PICKING_TARGET_HEIGHT_2

        # [-1, 0, 0, 0] 에 대한 trajectory를 구한 뒤 마지막 말단부만 joint변경하기 위해 아래 과정 수행
        self.move_group.set_pose_target(pose_goal)
        moving_plan = self.move_group.plan()
        target_endpoint = np.asarray(moving_plan[1].joint_trajectory.points[-1].positions)
        # 말단 부 Tip angle 계산 후 수정
        target_endpoint[-1] = self.TOOL_ORIENTATION

        print('다가가는중 .. ', pose_goal)
        self.move_group.go(target_endpoint, wait=True)
        print(' 포인트 1 도착 ')

        self.move_group.stop()
        self.move_group.clear_pose_targets()

        #self.goHome()

    def applyVaccum(self, OnOff):
        # OnOff = 1 : On
        self.SetIO(fun=1, pin=0, state=OnOff)

if __name__ == "__main__":
    robot_control = MoveRobotNode()
    robot_control.goHome()
    #robot_control.goPosition(0.453, 0.071, 0.25)
    rospy.spin()