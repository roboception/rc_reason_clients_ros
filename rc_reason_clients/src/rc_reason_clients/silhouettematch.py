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

import rospy

from math import sqrt
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Quaternion

from rc_reason_msgs.srv import SilhouetteMatchDetectObject
from rc_reason_msgs.srv import CalibrateBasePlane
from rc_reason_msgs.srv import GetBasePlaneCalibration
from rc_reason_msgs.srv import DeleteBasePlaneCalibration

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from rc_reason_clients.rest_client import RestClient


def instance_to_tf(instance):
    tf = TransformStamped()
    tf.header.frame_id = instance.pose_frame
    tf.child_frame_id = "{}_{}".format(instance.object_id, instance.id)
    tf.header.stamp = instance.timestamp
    tf.transform.translation.x = instance.pose.position.x
    tf.transform.translation.y = instance.pose.position.y
    tf.transform.translation.z = instance.pose.position.z
    tf.transform.rotation = instance.pose.orientation
    return tf


class SilhouetteMatchClient(RestClient):

    def __init__(self):
        super(SilhouetteMatchClient, self).__init__('rc_silhouettematch')

        # client only parameters
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.publish_markers = rospy.get_param("~publish_markers", True)

        self.pub_tf = rospy.Publisher('/tf', TFMessage, queue_size=10)
        self.pub_markers = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

        self.add_rest_service(SilhouetteMatchDetectObject, 'detect_object', self.detect_cb)
        self.add_rest_service(CalibrateBasePlane, 'calibrate_base_plane', self.calib_cb)
        self.add_rest_service(GetBasePlaneCalibration, 'get_base_plane_calibration', self.calib_cb)
        self.add_rest_service(DeleteBasePlaneCalibration, 'delete_base_plane_calibration', self.generic_cb)

    def generic_cb(self, srv_name, srv_type, request):
        response = self.call_rest_service(srv_name, srv_type, request)
        return response

    def detect_cb(self, srv_name, srv_type, request):
        response = self.call_rest_service(srv_name, srv_type, request)
        self.pub_instances(response.instances)
        return response

    def calib_cb(self, srv_name, srv_type, request):
        response = self.call_rest_service(srv_name, srv_type, request)
        if response.return_code.value >= 0:
            self.publish_base_plane_markers(response.plane, response.pose_frame)
        return response

    def pub_instances(self, instances):
        if not instances:
            return
        if not self.publish_tf:
            return
        transforms = [instance_to_tf(i) for i in instances]
        self.pub_tf.publish(TFMessage(transforms=transforms))

    def publish_base_plane_markers(self, plane, pose_frame):
        def create_marker(plane, id=0):
            m = Marker(action=Marker.ADD, type=Marker.CYLINDER)
            m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)
            m.header.frame_id = pose_frame
            m.pose.position.x = -plane.coef[0] * plane.coef[3]
            m.pose.position.y = -plane.coef[1] * plane.coef[3]
            m.pose.position.z = -plane.coef[2] * plane.coef[3]
            # quaternion yielding double the desired rotation (if normal is normalized):
            # q.w = dot(zaxis, normal), q.xyz = cross(zaxis, normal)
            # add quaternion with zero rotation (xyz=0, w=1) to get half the rotation from above
            # and normalize again
            q = Quaternion(x=-plane.coef[1], y=plane.coef[0], z=0.0, w=plane.coef[2] + 1.0)
            norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            m.pose.orientation.x = q.x / norm
            m.pose.orientation.y = q.y / norm
            m.pose.orientation.w = q.z / norm
            m.pose.orientation.x = q.z / norm
            m.scale.x = 1.0
            m.scale.y = 1.0
            m.scale.z = 0.001
            m.id = id
            m.ns = self.rest_name + "_base_plane"
            return m

        m = create_marker(plane)
        self.pub_markers.publish(MarkerArray(markers=[m]))


def main():
    client = SilhouetteMatchClient()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
