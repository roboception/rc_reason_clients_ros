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

from functools import partial

import rospy

import json
import re
import sys

import requests
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

from .message_converter import convert_dictionary_to_ros_message, convert_ros_message_to_dictionary

def requests_retry_session(retries=3,
                           backoff_factor=0.3,
                           status_forcelist=[429],
                           session=None):
    """"
    A requests session that will retry on TOO_MANY_REQUESTS.

    E.g. replace requests.get() with requests_retry_session().get()
    """
    session = session or requests.Session()
    retry = Retry(
        total=retries,
        read=retries,
        connect=retries,
        backoff_factor=backoff_factor,
        status_forcelist=status_forcelist,
    )
    adapter = HTTPAdapter(max_retries=retry)
    session.mount('http://', adapter)
    session.mount('https://', adapter)
    return session


class RestClient(object):

    def __init__(self, rest_name, ignored_parameters=[]):
        self.rest_name = rest_name
        self.ignored_parameters = ignored_parameters
        self.rest_services = []
        self.ddr = None

        rospy.init_node(rest_name + '_client', log_level=rospy.DEBUG)

        self.host = rospy.get_param('~host', '')
        if not self.host:
            rospy.logerr('host is not set')
            sys.exit(1)

        self._setup_ddr()

    def _get_rest_parameters(self):
        try:
            url = 'http://{}/api/v1/nodes/{}/parameters'.format(self.host, self.rest_name)
            res = requests_retry_session().get(url)
            if res.status_code != 200:
                rospy.logerr("Getting parameters failed with status code: %d", res.status_code)
                return []
            return res.json()
        except Exception as e:
            rospy.logerr(str(e))
            return []

    def _set_rest_parameters(self, parameters):
        try:
            url = 'http://{}/api/v1/nodes/{}/parameters'.format(self.host, self.rest_name)
            res = requests_retry_session().put(url, json=parameters)
            j = res.json()
            rospy.logdebug("set parameters response: %s", json.dumps(j, indent=2))
            if 'return_code' in j and j['return_code']['value'] != 0:
                rospy.logwarn("Setting parameter failed: %s", j['return_code']['message'])
                return []
            if res.status_code != 200:
                rospy.logerr("Setting parameters failed with status code: %d", res.status_code)
                return []
            return j
        except Exception as e:
            rospy.logerr(str(e))
            return []

    def _setup_ddr(self):
        self.ddr = DDynamicReconfigure(rospy.get_name())
        rest_params = [p for p in self._get_rest_parameters() if p['name'] not in self.ignored_parameters]

        def enum_method_from_param(p):
            if p['type'] != 'string':
                return ""
            enum_matches = re.findall(r'.*\[(?P<enum>.+)\].*', p['description'])
            if not enum_matches:
                return ""
            enum_names = [str(e.strip()) for e in enum_matches[0].split(',')]
            enum_list = [self.ddr.const(n, 'str', n, n) for n in enum_names]
            return self.ddr.enum(enum_list, p['name'] + '_enum')

        for p in rest_params:
            level = 0
            edit_method = enum_method_from_param(p)
            if p['type'] == 'int32':
                self.ddr.add(p['name'], 'int', level, p['description'], p['default'], p['min'], p['max'])
            elif p['type'] == 'float64':
                self.ddr.add(p['name'], 'double', level, p['description'], p['default'], p['min'], p['max'])
            elif p['type'] == 'string':
                self.ddr.add(p['name'], 'str', level, p['description'], str(p['default']), edit_method=edit_method)
            elif p['type'] == 'bool':
                self.ddr.add(p['name'], 'bool', level, p['description'], p['default'])
            else:
                rospy.logwarn("Unsupported parameter type: %s", p['type'])

        self.ddr.start(self._dyn_rec_callback)

    def _dyn_rec_callback(self, config, level):
        rospy.logdebug("Received reconf call: " + str(config))
        new_rest_params = [{'name': n, 'value': config[n]} for n in self.ddr.get_variable_names() if n in config]
        if new_rest_params:
            returned_params = self._set_rest_parameters(new_rest_params)
            for p in returned_params:
                if p['name'] not in config:
                    rospy.logerr("param %s not in config", p['name'])
                    continue
                config[p['name']] = p['value']
        return config

    def call_rest_service(self, name, srv_type=None, request=None):
        try:
            args = {}
            if request is not None:
                # convert ROS request to JSON (with custom API mappings)
                args = convert_ros_message_to_dictionary(request)
                rospy.logdebug('calling {} with args: {}'.format(name, args))

            url = 'http://{}/api/v1/nodes/{}/services/{}'.format(self.host, self.rest_name, name)
            res = requests_retry_session().put(url, json={'args': args})

            j = res.json()
            rospy.logdebug("{} rest response: {}".format(name, json.dumps(j, indent=2)))
            rc = j['response'].get('return_code')
            if rc is not None and rc['value'] < 0:
                rospy.logwarn("service {} returned an error: [{}] {}".format(name, rc['value'], rc['message']))

            # convert to ROS response
            if srv_type is not None:
                response = convert_dictionary_to_ros_message(srv_type._response_class(), j['response'])
            else:
                response = j['response']
        except Exception as e:
            rospy.logerr(str(e))
            if srv_type is not None:
                response = srv_type._response_class()
                if hasattr(response, 'return_code'):
                    response.return_code.value = -1000
                    response.return_code.message = str(e)
        return response

    def add_rest_service(self, srv_type, srv_name, callback):
        """create a service and inject the REST-API service name"""
        srv = rospy.Service(rospy.get_name() + "/" + srv_name, srv_type, partial(callback, srv_name, srv_type))
        self.rest_services.append(srv)
