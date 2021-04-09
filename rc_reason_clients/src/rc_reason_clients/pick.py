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

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

from rc_reason_msgs.srv import SetLoadCarrier, GetLoadCarriers, DeleteLoadCarriers
from rc_reason_msgs.srv import SetRegionOfInterest3D, GetRegionsOfInterest3D, DeleteRegionsOfInterest3D
from rc_reason_msgs.srv import DetectLoadCarriers, DetectFillingLevel
from rc_reason_msgs.srv import ComputeGrasps, DetectItems

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from rc_reason_clients.rest_client import RestClient


def load_carrier_to_tf(lc, postfix):
    tf = TransformStamped()
    tf.header.frame_id = lc.pose.header.frame_id
    tf.child_frame_id = "lc_{}".format(postfix)
    tf.header.stamp = lc.pose.header.stamp
    tf.transform.translation.x = lc.pose.pose.position.x
    tf.transform.translation.y = lc.pose.pose.position.y
    tf.transform.translation.z = lc.pose.pose.position.z
    tf.transform.rotation = lc.pose.pose.orientation
    return tf


def grasp_to_tf(grasp, postfix):
    tf = TransformStamped()
    tf.header.frame_id = grasp.pose.header.frame_id
    tf.child_frame_id = "grasp_".format(postfix)
    tf.header.stamp = grasp.pose.header.stamp
    tf.transform.translation.x = grasp.pose.pose.position.x
    tf.transform.translation.y = grasp.pose.pose.position.y
    tf.transform.translation.z = grasp.pose.pose.position.z
    tf.transform.rotation = grasp.pose.pose.orientation
    return tf


def item_to_tf(item, postfix):
    tf = TransformStamped()
    tf.header.frame_id = item.pose.header.frame_id
    tf.child_frame_id = "boxitem_".format(postfix)
    tf.header.stamp = item.pose.header.stamp
    tf.transform.translation.x = item.pose.pose.position.x
    tf.transform.translation.y = item.pose.pose.position.y
    tf.transform.translation.z = item.pose.pose.position.z
    tf.transform.rotation = item.pose.pose.orientation
    return tf


def lc_to_marker(lc, lc_no, ns):
    m = Marker(action=Marker.ADD, type=Marker.CUBE)
    m.color = ColorRGBA(r=0.0, g=0.2, b=0.8, a=0.3)
    m.header = lc.pose.header
    m.ns = ns

    # FIXME: calculate actual bottom and sides
    m.id = lc_no
    m.pose = lc.pose.pose
    m.scale.x = lc.outer_dimensions.x
    m.scale.y = lc.outer_dimensions.y
    m.scale.z = lc.outer_dimensions.z

    return m


class PickClient(RestClient):

    def __init__(self, rest_name):
        super(PickClient, self).__init__(rest_name)

        # client only parameters
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.publish_markers = rospy.get_param("~publish_markers", True)

        self.pub_tf = rospy.Publisher('/tf', TFMessage, queue_size=10)
        self.pub_markers = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

        self.grasp_markers = []
        self.lc_markers = []

        rospy.on_shutdown(self.stop)

        self.start()

        self.add_rest_service(SetLoadCarrier, 'set_load_carrier', self.generic_cb)
        self.add_rest_service(GetLoadCarriers, 'get_load_carriers', self.generic_cb)
        self.add_rest_service(DeleteLoadCarriers, 'delete_load_carriers', self.generic_cb)
        self.add_rest_service(SetRegionOfInterest3D, 'set_region_of_interest', self.generic_cb)
        self.add_rest_service(GetRegionsOfInterest3D, 'get_regions_of_interest', self.generic_cb)
        self.add_rest_service(DeleteRegionsOfInterest3D, 'delete_regions_of_interest', self.generic_cb)
        self.add_rest_service(DetectLoadCarriers, 'detect_load_carriers', self.lc_cb)
        self.add_rest_service(DetectFillingLevel, 'detect_filling_level', self.lc_cb)

    def start(self):
        rospy.loginfo("starting %s", self.rest_name)
        self.call_rest_service('start')

    def stop(self):
        rospy.loginfo("stopping %s", self.rest_name)
        self.call_rest_service('stop')

    def generic_cb(self, srv_name, srv_type, request):
        response = self.call_rest_service(srv_name, srv_type, request)
        return response

    def lc_cb(self, srv_name, srv_type, request):
        response = self.call_rest_service(srv_name, srv_type, request)
        self.publish_lcs(response.load_carriers)
        return response

    def publish_lcs(self, lcs):
        if lcs and self.publish_tf:
            transforms = [load_carrier_to_tf(lc, i) for i, lc in enumerate(lcs)]
            self.pub_tf.publish(TFMessage(transforms=transforms))
        if self.publish_markers:
            self.publish_lc_markers(lcs)

    def publish_grasps(self, grasps):
        if grasps and self.publish_tf:
            transforms = [grasp_to_tf(grasp, i) for i, grasp in enumerate(grasps)]
            self.pub_tf.publish(TFMessage(transforms=transforms))
        if self.publish_markers:
            self.publish_grasps_markers(grasps)

    def publish_grasps_markers(self, grasps):
        def create_marker(grasp, id):
            m = Marker(action=Marker.ADD, type=Marker.SPHERE)
            m.color = ColorRGBA(r=0.8, g=0.2, b=0.0, a=0.8)
            m.scale.x = grasp.max_suction_surface_length
            m.scale.y = grasp.max_suction_surface_width
            m.scale.z = 0.001
            m.header = grasp.pose.header
            m.pose = grasp.pose.pose
            m.id = i
            m.ns = self.rest_name + "_grasps"
            return m

        new_markers = []
        for i, grasp in enumerate(grasps):
            m = create_marker(grasp, i)
            if i < len(self.grasp_markers):
                self.grasp_markers[i] = m
            else:
                self.grasp_markers.append(m)
            new_markers.append(m)
        for i in range(len(grasps), len(self.grasp_markers)):
            # delete old markers
            self.grasp_markers[i].action = Marker.DELETE
        self.pub_markers.publish(MarkerArray(markers=self.grasp_markers))
        self.grasp_markers = new_markers

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


class ItemPickClient(PickClient):

    def __init__(self, rest_name):
        super(ItemPickClient, self).__init__(rest_name)
        self.add_rest_service(ComputeGrasps, 'compute_grasps', self.compute_grasps_cb)

    def compute_grasps_cb(self, srv_name, srv_type, request):
        response = self.call_rest_service(srv_name, srv_type, request)
        self.publish_lcs(response.load_carriers)
        self.publish_grasps(response.grasps)
        return response


class BoxPickClient(PickClient):

    def __init__(self, rest_name):
        super(BoxPickClient, self).__init__(rest_name)
        self.add_rest_service(ComputeGrasps, 'compute_grasps', self.compute_grasps_cb)
        self.add_rest_service(DetectItems, 'detect_items', self.detect_items_cb)

    def compute_grasps_cb(self, srv_name, srv_type, request):
        response = self.call_rest_service(srv_name, srv_type, request)
        self.publish_lcs(response.load_carriers)
        self.publish_grasps(response.grasps)
        self.publish_items(response.items)
        return response

    def detect_items_cb(self, srv_name, srv_type, request):
        response = self.call_rest_service(srv_name, srv_type, request)
        self.publish_lcs(response.load_carriers)
        self.publish_items(response.items)
        return response

    def publish_items(self, items):
        if not items:
            return
        if not self.publish_tf:
            return
        transforms = [item_to_tf(item, i) for i, item in enumerate(items)]
        self.pub_tf.publish(TFMessage(transforms=transforms))


def main(rest_node='rc_itempick'):
    if rest_node == 'rc_itempick':
        client = ItemPickClient(rest_node)
    elif rest_node == 'rc_boxpick':
        client = BoxPickClient(rest_node)
    else:
        import logging
        logging.error('unknown rest_node %s', rest_node)
        exit(1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


def rc_itempick_client():
    main(rest_node='rc_itempick')


def rc_boxpick_client():
    main(rest_node='rc_boxpick')


if __name__ == '__main__':
    main()
