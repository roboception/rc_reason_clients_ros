#!/usr/bin/env python

# Copyright 2020 Roboception GmbH
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

import sys

from rc_reason_msgs.srv import HandEyeCalibration, HandEyeCalibrationRequest
from rc_reason_msgs.srv import HandEyeCalibrationTrigger
from rc_reason_msgs.srv import SetHandEyeCalibration
from rc_reason_msgs.srv import SetHandEyeCalibrationPose

from rest_client import RestClient


class HandEyeCalibClient(RestClient):

    def __init__(self, host):
        super(HandEyeCalibClient, self).__init__('rc_hand_eye_calibration', host)

        self.add_rest_service(HandEyeCalibration, 'calibrate', self.pub_cb)
        self.add_rest_service(HandEyeCalibration, 'get_calibration', self.pub_cb)
        self.add_rest_service(SetHandEyeCalibration, 'set_calibration', self.pub_cb)
        self.add_rest_service(HandEyeCalibrationTrigger, 'save_calibration', self.generic_cb)
        self.add_rest_service(HandEyeCalibrationTrigger, 'delete_calibration', self.generic_cb)
        self.add_rest_service(HandEyeCalibrationTrigger, 'reset_calibration', self.generic_cb)
        self.add_rest_service(SetHandEyeCalibrationPose, 'set_pose', self.generic_cb)

        # get initial calibration from sensor
        self.pub_cb('get_calibration', HandEyeCalibration, HandEyeCalibrationRequest())

    def generic_cb(self, srv_name, srv_type, request):
        response = self.call_rest_service(srv_name, srv_type, request)
        if not response.success:
            rospy.logwarn("service %s: %s", srv_name, response.message)
        return response

    def pub_cb(self, srv_name, srv_type, request):
        """Handle service call and publish hand-eye-calib if successful"""
        response = self.call_rest_service(srv_name, srv_type, request)
        if response.success:
            self.pub_hand_eye(response.pose, response.robot_mounted)
        else:
            rospy.logwarn("service %s: %s", srv_name, response.message)
        return response

    def pub_hand_eye(self, pose, robot_mounted):
        pass


def main():
    rospy.init_node('hand_eye_calib_client', log_level=rospy.DEBUG)

    host = rospy.get_param('~host', '')
    if not host:
        rospy.logerr('host is not set')
        sys.exit(1)

    client = HandEyeCalibClient(host)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
