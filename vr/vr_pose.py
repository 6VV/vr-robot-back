#!/usr/bin/env python
# -*- coding: utf-8 -*-

import openvr
import triad_openvr


class VRPose:
    def __init__(self):
        v = triad_openvr.triad_openvr()
        self.hands = [v.devices['controller_1'], v.devices['controller_2']]

    def get_pose_euler(self, index):
        return self.hands[index].get_pose_euler()

    def get_pose_quaternion(self, index):
        return self.hands[index].get_pose_quaternion()

    def getControllerStateWithPose(self, index):
        device = self.hands[index]
        _, state, pTrackedDevicePose = device.vr.getControllerStateWithPose(
            openvr.TrackingUniverseStanding, device.index)
        touched = {
            'circleX': state.rAxis[0].x, 'circleY': state.rAxis[0].y, 'gripper': state.rAxis[1].x}
        euler_pose = triad_openvr.convert_to_euler(
            pTrackedDevicePose.mDeviceToAbsoluteTracking)
        quaternion_pose = triad_openvr.convert_to_quaternion(
            pTrackedDevicePose.mDeviceToAbsoluteTracking)
        return {'touched': touched, 'euler_pose': euler_pose, 'quaternion_pose': quaternion_pose}
