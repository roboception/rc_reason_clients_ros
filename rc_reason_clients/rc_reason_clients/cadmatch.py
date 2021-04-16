#!/usr/bin/env python

# Copyright 2021 Roboception GmbH
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

from __future__ import absolute_import

import rospy

from math import sqrt
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Quaternion

from rc_reason_msgs.srv import CadMatchDetectObject

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from .rest_client import RestClient
from .transform_helpers import lc_to_marker, load_carrier_to_tf, match_to_tf


class CadMatchClient(RestClient):

    def __init__(self):
        ignored_parameters = ['load_carrier_crop_distance', 'load_carrier_model_tolerance']
        super(CadMatchClient, self).__init__('rc_cadmatch', ignored_parameters)

        # client only parameters
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.publish_markers = rospy.get_param("~publish_markers", True)

        self.pub_tf = rospy.Publisher('/tf', TFMessage, queue_size=10)
        self.pub_markers = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

        self.lc_markers = []

        self.add_rest_service(CadMatchDetectObject, 'detect_object', self.detect_cb)

        rospy.on_shutdown(self.stop)

        self.start()

    def start(self):
        rospy.loginfo("starting %s", self.rest_name)
        self.call_rest_service('start')

    def stop(self):
        rospy.loginfo("stopping %s", self.rest_name)
        self.call_rest_service('stop')

    def detect_cb(self, srv_name, srv_type, request):
        response = self.call_rest_service(srv_name, srv_type, request)
        self.pub_matches(response.matches)
        self.publish_lcs(response.load_carriers)
        return response

    def pub_matches(self, matches):
        if not matches or not self.publish_tf:
            return
        transforms = [match_to_tf(i) for i in matches]
        self.pub_tf.publish(TFMessage(transforms=transforms))

    def publish_lcs(self, lcs):
        if lcs and self.publish_tf:
            transforms = [load_carrier_to_tf(lc, i) for i, lc in enumerate(lcs)]
            self.pub_tf.publish(TFMessage(transforms=transforms))
        if self.publish_markers:
            self.publish_lc_markers(lcs)

    def publish_lc_markers(self, lcs):
        new_markers = []
        for i, lc in enumerate(lcs):
            m = lc_to_marker(lc, i, self.rest_name + "_lcs")
            if i < len(self.lc_markers):
                self.lc_markers[i] = m
            else:
                self.lc_markers.append(m)
            new_markers.append(m)
        for i in range(len(lcs), len(self.lc_markers)):
            # delete old markers
            self.lc_markers[i].action = Marker.DELETE
        self.pub_markers.publish(MarkerArray(markers=self.lc_markers))
        self.lc_markers = new_markers


def main():
    client = CadMatchClient()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
