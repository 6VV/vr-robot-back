#!/usr/bin/env python
# -- coding: utf-8 --

import numpy
from tf import transformations, TransformListener
import rospy
import geometry_msgs
import math


class TransformerTool:
    def __init__(self, target_frame=None, source_frame=None):
        self.target_frame = target_frame
        self.source_frame = source_frame
        if target_frame is not None and source_frame is not None:
            self.mat44 = self.asMatrix(
                target_frame=target_frame, source_frame=source_frame)
            self.mat44Reserver = self.asMatrix(
                target_frame=source_frame, source_frame=target_frame)

    def quat2rvec(self, quat):
        '四元数=>旋转角'
        theta = math.acos(quat[3]) * 2
        if theta < 0.001:
            return [0, 0, 0]
        else:
            axis = [x / math.sin(theta) for x in quat[0:3]]
            norm = math.sqrt(axis[0] * axis[0] + axis[1]
                             * axis[1] + axis[2] * axis[2])
            rvec = [x * theta / norm for x in axis]
            return rvec

    def rvec2quat(self, rvec):
        '旋转角=>四元数'
        theta = math.sqrt(rvec[0] * rvec[0] + rvec[1]
                          * rvec[1] + rvec[2] * rvec[2])
        if theta < 0.001:
            return [0, 0, 0, 1]
        else:
            axis = [x / theta for x in rvec]
            sht = math.sin(theta * 0.5)
            quat = [x * sht for x in axis]
            quat.append(math.cos(theta * 0.5))
            return quat

    def transformPoseWithFrame(self, target_frame, source_frame, pose):
        '位姿在不同坐标系下的变换'
        mat44 = self.asMatrix(target_frame=target_frame,
                              source_frame=source_frame)
        return self._transformPose(mat44=mat44, pose=pose)

    def transformPose(self, pose):
        return self._transformPose(mat44=self.mat44, pose=pose)

    def _transformPose(self, mat44, pose):
        pose44 = numpy.dot(self.xyz_to_mat44(pose.position),
                           self.xyzw_to_mat44(pose.orientation))
        txpose = numpy.dot(mat44, pose44)
        # print(txpose)

        xyz = tuple(transformations.translation_from_matrix(txpose))[:3]
        quat = tuple(self.quaternion_from_matrix(txpose))
        # print(quat)

        return geometry_msgs.msg.Pose(geometry_msgs.msg.Point(*xyz), geometry_msgs.msg.Quaternion(*quat))

    def asMatrix(self, target_frame, source_frame):
        tran = TransformListener()
        tran.waitForTransform(
            target_frame=target_frame, source_frame=source_frame, time=rospy.Time(0), timeout=rospy.Duration(4.0))
        translation, rotation = tran.lookupTransform(target_frame=target_frame,
                                                     source_frame=source_frame, time=rospy.Time(0))

        return self.fromTranslationRotation(translation, rotation)

    def fromTranslationRotation(self, translation, rotation):
        return numpy.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))

    def xyz_to_mat44(self, pos):
        return transformations.translation_matrix((pos.x, pos.y, pos.z))

    def xyzw_to_mat44(self, ori):
        return transformations.quaternion_matrix((ori.x, ori.y, ori.z, ori.w))

    def transformQuaternion(self, quaternion):
        return self._transformQuaternion(self.mat44, quaternion)

    def transformQuaternionWithFrame(self, target_frame, source_frame, quaternion):
        mat44 = self.asMatrix(target_frame=target_frame,
                              source_frame=source_frame)
        return self._transformQuaternion(mat44, quaternion)

    def _transformQuaternion(self, mat44, quaternion):
        pose44 = self.xyzw_to_mat44(quaternion)

        txpose = numpy.dot(mat44, pose44)

        # TODO:修改转换矩阵
        # quat = tuple(transformations.quaternion_from_matrix(txpose))
        quat = tuple(self.quaternion_from_matrix(txpose))

        return geometry_msgs.msg.Quaternion(*quat)

    def quaternion_from_matrix(self,matrix):
        """
        自定义转换矩阵，用于替代tf相关函数，避免突变，
        采用tf变换函数时，当右臂旋转到一定角度后，出现较大幅度变化
        暂不能确定是否会出现其它问题
        """
        q = numpy.empty((4, ), dtype=numpy.float64)
        M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:4, :4]
        t = numpy.trace(M)
        # if t > M[3, 3]:
        q[3] = t
        q[2] = M[1, 0] - M[0, 1]
        q[1] = M[0, 2] - M[2, 0]
        q[0] = M[2, 1] - M[1, 2]
        # else:
        #     i, j, k = 0, 1, 2
        #     if M[1, 1] > M[0, 0]:
        #         i, j, k = 1, 2, 0
        #     if M[2, 2] > M[i, i]:
        #         i, j, k = 2, 0, 1
        #     t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
        #     q[i] = -t
        #     q[j] = -(M[i, j] + M[j, i])
        #     q[k] = -(M[k, i] + M[i, k])
        #     q[3] = -(M[k, j] - M[j, k])
        q *= 0.5 / math.sqrt(t * M[3, 3])
        return q
