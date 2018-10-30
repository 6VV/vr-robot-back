#!/usr/bin/env python
# --coding: utf-8 --

'''
此代码已废弃
'''

import rospy
from geometry_msgs.msg import Pose
import moveit_commander
from urx import URRobot
from urx.urscript import URScript
import time
from TransformerTool import TransformerTool
import numpy as np
import tf
import math
from std_msgs.msg import Int32, Float32MultiArray
import threading


def mat2rvec(mat):
    theta = math.acos((mat[0, 0] + mat[1, 1] + mat[2, 2] - 1.0) * 0.5)
    if theta < 0.001:
        return [0, 0, 0]
    else:
        rx = theta * (mat[2, 1] - mat[1, 2]) / (2 * math.sin(theta))
        ry = theta * (mat[0, 2] - mat[2, 0]) / (2 * math.sin(theta))
        rz = theta * (mat[1, 0] - mat[0, 1]) / (2 * math.sin(theta))
        return [rx, ry, rz]


class VRController:
    def __init__(self, host, ur_frame, base_frame, arm_name, init_posej=None):
        try:
            self.vr_init_pose = None
            self.ur_init_pose = None
            self.time = time.time()
            self.scale = 1  # 机器人移动距离与手臂的比值
            self.ur_frame = ur_frame
            self.base_frame = base_frame
            # self.angle = None
            self.last_gripper_value = 0
            self.stop_vr = False
            self.gripper = [False, False]
            self.force_mode = [False, False]

            self.tran_base_to_ur = TransformerTool(
                target_frame=ur_frame, source_frame=base_frame)
            self.ur_robot = URRobot(host)

            if init_posej is not None:
                self.ur_robot.movej(init_posej, acc=0.5, vel=1)

            rospy.Subscriber(
                name=arm_name + "/vr_pose4", data_class=Pose, callback=self.on_sub_pose, queue_size=3)
            rospy.Subscriber(name=arm_name + "/gripper", data_class=Int32,
                             callback=self.on_sub_gripper, queue_size=1)
        except Exception, e:
            print(e)
            self.ur_robot.close()
            exit()

    def close(self):
        self.ur_robot.close()

    def set_force_mode(self, value):
        if value == self.force_mode[1]:
            self.force_mode[0] = False
        else:
            self.force_mode[1] = value
            self.force_mode[0] = True

    def on_sub_gripper(self, data):
        if data.data == self.last_gripper_value:
            self.gripper[0] = False
            return

        self.gripper[0] = True
        self.gripper[1] = data.data != 1

        # self.ur_robot.set_digital_out(8, data.data != 1)
        self.last_gripper_value = data.data

    def on_sub_pose(self, data):
        try:
            current_pose = self.rvec_pose_from_msg_pose(data)

            ###########################################
            '调整世界坐标系下的角度'
            rotation_matrix = tf.transformations.quaternion_matrix(
                [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
            target_rotation_matrix = np.dot(rotation_matrix, np.array([[0, 0, 1, 0],
                                                                       [1, 0, 0, 0],
                                                                       [0, 1, 0, 0],
                                                                       [0, 0, 0, 1]]))
            angle = mat2rvec(target_rotation_matrix)

            # if self.ur_frame == "right_arm_base":
            #     self.angle[2] -= 180 / math.pi

            current_pose[3] = angle[0]
            current_pose[4] = angle[1]
            current_pose[5] = angle[2]

            # self.angle = current_pose[3:6]
            ################################################

            if self.stop_vr:
                self.vr_init_pose = None
                self.ur_robot.send_program(
                    'servoj(get_actual_joint_positions(),t=0.05,lookahead_time=0.03,gain=100)')
                return
            elif self.vr_init_pose is None:
                self.vr_init_pose = current_pose

                pose = self.ur_robot.getl()
                self.ur_init_pose = self.tranform_pose(TransformerTool(
                    target_frame=self.base_frame, source_frame=self.ur_frame), pose=pose)

            target_pose = []
            for i in range(0, 3):
                target_pose.append(
                    self.ur_init_pose[i] + (current_pose[i] - self.vr_init_pose[i]) * self.scale)

            target_pose.append(angle[0])
            target_pose.append(angle[1])
            target_pose.append(angle[2])

            target_pose = self.tranform_pose(
                self.tran_base_to_ur, pose=target_pose)

            self.move_arm_robot(target_pose)

        except Exception, e:
            print(e)
            # self.ur_robot.close()

    def rvec_pose_from_msg_pose(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w)
        # tran = TransformerTool()
        rvec = self.tran_base_to_ur.quat2rvec(quaternion)
        return [pose.position.x, pose.position.y, pose.position.z, rvec[0], rvec[1], rvec[2]]

    def stop_control(self):
        # self.ur_robot.stopj()
        self.stop_vr = True

    def begin_control(self):
        self.stop_vr = False

    def move_arm_robot(self, target_pose):
        time_limit = 0.25
        joing_speed_limit = [30.0, 30.0, 60.0, 90.0, 90.0, 120.0]
        joint_max_distance = []
        for i in range(len(joing_speed_limit)):
            joint_max_distance.append(
                joing_speed_limit[i] / 180 * math.pi * time_limit)

        # 当工具距离机械臂基座标系较远时停止移动（此方法治标不治本，更合理的方式应为无法规划出有效路径时停止移动，暂未找到判断是否成功规划路径的方法）
        if math.sqrt(math.pow(target_pose[0], 2) + math.pow(target_pose[1], 2) + math.pow(target_pose[2], 2) > 0.65):
            self.ur_robot.send_program(
                'servoj(get_actual_joint_positions(),t={},lookahead_time=0.1,gain=100)'.format(time_limit))
            print('too long')
            return

        urscript = URScript()

        # 控制夹手
        if self.gripper[0]:
            urscript.add_line_to_program(
                'set_tool_digital_out(0,{})'.format(self.gripper[1]))
        # urscript.add_line_to_program(
        #     'textmsg("-------------------------------------------------")')

        urscript.add_line_to_program('pi={}'.format(math.pi))  # 添加pi常量

        # urscript.add_line_to_program('textmsg("pose: [{},{},{},{},{},{}])")'.format(
        #     target_pose[0], target_pose[1], target_pose[2], target_pose[3], target_pose[4], target_pose[5]))

        # 获取当前关节角度
        urscript.add_line_to_program(
            'current_joint=get_actual_joint_positions()')

        # 力模式
        # if self.force_mode[0]:
        #     if self.force_mode[1]:
        #         urscript.add_line_to_program(
        #             'force_mode(target_pose,[0,0,0,1.1,1.1,1.1],[0,0,0,0,0,0],2,[100,100,100,20,20,20])')
        #         urscript.add_line_to_program('sleep(0.1)')
        #     else:
        #         urscript.add_line_to_program('end_force_mode()')

        urscript.add_line_to_program('inverse_kin=current_joint')
        urscript.add_line_to_program('inverse_kin=get_inverse_kin(p[{},{},{},{},{},{}],maxPositionError=0.02,maxOrientationError=0.02)'.format(
            target_pose[0], target_pose[1], target_pose[2], target_pose[3], target_pose[4], target_pose[5]))

        # urscript.add_line_to_program('textmsg("inverse kin:",inverse_kin)')

        # 若计算出的关节值与当前值差距较大,保持当前状态(仅判断前几个关节，治标不治本)
        urscript.add_line_to_program('i=0')
        urscript.add_line_to_program('while i<3:')
        urscript.add_line_to_program(
            '  if norm(inverse_kin[i]-current_joint[i])>1:')
        urscript.add_line_to_program('    inverse_kin=current_joint')
        # urscript.add_line_to_program('    textmsg(i,": distance too big")')
        urscript.add_line_to_program('    break')
        urscript.add_line_to_program('  end')
        urscript.add_line_to_program('  i=i+1')
        urscript.add_line_to_program('end')

        # 添加速度限制
        urscript.add_line_to_program('joint_max_distance=[{},{},{},{},{},{}]'.format(
            joint_max_distance[0], joint_max_distance[1], joint_max_distance[2], joint_max_distance[3], joint_max_distance[4], joint_max_distance[5]))

        # urscript.add_line_to_program('textmsg("current joint:",current_joint)')

        # 添加关节限制
        urscript.add_line_to_program('i=0')
        urscript.add_line_to_program('while i<6:')

        # 添加关节速度限制
        urscript.add_line_to_program(
            '  if inverse_kin[i]-current_joint[i]>joint_max_distance[i]:')
        urscript.add_line_to_program(
            '    inverse_kin[i]=current_joint[i]+joint_max_distance[i]')
        urscript.add_line_to_program(
            '  elif inverse_kin[i]-current_joint[i]<-joint_max_distance[i]:')
        urscript.add_line_to_program(
            '    inverse_kin[i]=current_joint[i]-joint_max_distance[i]')
        urscript.add_line_to_program('  end')

        # 添加关节角度限制
        urscript.add_line_to_program(
            '  if i<5 and norm(inverse_kin[i])>{}:'.format(math.pi * 2 - 0.2))
        urscript.add_line_to_program('    inverse_kin[i]=current_joint[i]')
        # urscript.add_line_to_program('    textmsg("***: ",i)')
        urscript.add_line_to_program('  end')

        urscript.add_line_to_program('  i=i+1')
        urscript.add_line_to_program('end')

        # urscript.add_line_to_program('textmsg("target joint:",inverse_kin)')

        # urscript.add_line_to_program(
        #     'if norm(inverse_kin[0])<pi*2 and norm(inverse_kin[1])<pi*2:')
        urscript.add_line_to_program(
            'servoj(inverse_kin,t={},lookahead_time=0.05,gain=100)'.format(time_limit))
        # urscript.add_line_to_program('end')

        prog = urscript()
        # prog = "servoj(get_inverse_kin(p[{},{},{},{},{},{}]),t=0.30,lookahead_time=0.00,gain=100)".format(
        #     target_pose[0], target_pose[1], target_pose[2], target_pose[3], target_pose[4], target_pose[5])

        self.ur_robot.secmon.send_program(prog)

    def tranform_pose(self, tran, pose):
        src_quat = tran.rvec2quat(pose[3:6])

        src_pose = Pose()
        src_pose.position.x = pose[0]
        src_pose.position.y = pose[1]
        src_pose.position.z = pose[2]
        src_pose.orientation.x = src_quat[0]
        src_pose.orientation.y = src_quat[1]
        src_pose.orientation.z = src_quat[2]
        src_pose.orientation.w = src_quat[3]

        target_pose = tran.transformPose(pose=src_pose)

        result = self.rvec_pose_from_msg_pose(target_pose)

        ##################################################
        # if self.ur_frame == "right_arm_base":
        #     print([target_pose.orientation.x, target_pose.orientation.y,
        #            target_pose.orientation.z, target_pose.orientation.w])
        #     if result[5] < -3:
        #         target_pose.orientation.x *= -1
        #         target_pose.orientation.y *= -1
        #         target_pose.orientation.z *= -1
        #         target_pose.orientation.w *= -1
        #         inverse_pose = self.rvec_pose_from_msg_pose(target_pose)
        #         print("inverse rvec: " + str(inverse_pose[3:6]))
        #         return inverse_pose
        ##################################################

        return result


# def init_pose():
#     left_ur_robot = None
#     right_ur_robot = None
#     try:
#         left_ur_robot = URRobot("192.168.31.63")
#         # left_ur_robot.movej(, vel=1, acc=0.5)
#         right_ur_robot = URRobot("192.168.31.64")
#         right_ur_robot.movej(, vel=1, acc=0.5)
#     finally:
#         if left_ur_robot is not None:
#             left_ur_robot.close()
#         if right_ur_robot is not None:
#             right_ur_robot.close()


def on_left_area_callback(data):
    global left_vr_controller, right_vr_controller
    if data.data[1] < 0:
        if left_vr_controller is not None:
            left_vr_controller.stop_control()
        if right_vr_controller is not None:
            right_vr_controller.stop_control()
    else:
        if left_vr_controller is not None:
            left_vr_controller.begin_control()
        if right_vr_controller is not None:
            right_vr_controller.begin_control()

    if data.data[1] > 0:
        if left_vr_controller is not None:
            left_vr_controller.set_force_mode(True)
        if right_vr_controller is not None:
            right_vr_controller.set_force_mode(True)
    else:
        if left_vr_controller is not None:
            left_vr_controller.set_force_mode(False)
        if right_vr_controller is not None:
            right_vr_controller.set_force_mode(False)


if __name__ == "__main__":
    # init_pose()
    left_vr_controller = None
    right_vr_controller = None

    try:
        rospy.init_node(name="ur_robot_controller", anonymous=True)

        rospy.Subscriber(name="/left_vr/area", data_class=Float32MultiArray,
                         callback=on_left_area_callback, queue_size=10)

        left_init_posej = [84.37 / 180 * math.pi, -57.39 / 180 * math.pi, 139.39 / 180 *
                           math.pi, -138.75 / 180 * math.pi, 100.11 / 180 * math.pi, 345.21 / 180 * math.pi]
        left_vr_controller = VRController(host="192.168.31.63", ur_frame="left_arm_base",
                                          base_frame="frame_base", arm_name="/left_vr", init_posej=left_init_posej)

        right_init_posej = [-106.68 / 180 * math.pi, -133.58 / 180 * math.pi, -130.70 / 180 *
                            math.pi, -30.22 / 180 * math.pi, -97.49 / 180 * math.pi, (9.18) / 180 * math.pi]
        right_vr_controller = VRController(host="192.168.31.64", ur_frame="right_arm_base",
                                           base_frame="frame_base", arm_name="/right_vr", init_posej=right_init_posej)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()
    finally:
        if left_vr_controller is not None:
            left_vr_controller.close()
        if right_vr_controller is not None:
            right_vr_controller.close()
