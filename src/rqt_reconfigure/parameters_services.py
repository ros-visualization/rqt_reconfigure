# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Gonzalo de Pedro

from rcl_interfaces.srv import ListParameters
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.srv import DescribeParameters
from rcl_interfaces.msg import Parameter
import json


class ParametersServices(object):

    def __init__(self, node):
        self._node = node

    def create_parameters_service_client(self, name, timeout=None, config_callback=None, description_callback=None):
        return ParametersServiceClient(name, self._node, timeout, config_callback, description_callback)

    def find_reconfigure_services(self):
        return self._node.get_node_names()


class ParametersServiceClient(object):
    """
    Python dynamic_reconfigure client API
    """
    def __init__(self, name, node, timeout=None, config_callback=None, description_callback=None):
        """
        """

        self._node = node
#Name should be the client_node_name used below
        self._get_params_client = node.create_client(
            GetParameters,
            '{name}/get_parameters'.format_map(locals()))
        self._set_params_client = node.create_client(
            SetParameters,
            '{name}/set_parameters'.format_map(locals()))
        self._list_params_client = node.create_client(
            ListParameters,
            '{name}/list_parameters'.format_map(locals()))
        self._describe_params_client = node.create_client(
            DescribeParameters,
            '{name}/describe_parameters'.format_map(locals()))

    def get_group_descriptions(self):
        resp = self._list_params_client.call(ListParameters.Request())
        req = DescribeParameters.Request()
        req.names = resp.result.names
        print(":: NAMES : " + str(req.names))
        resp = self._describe_params_client.call(req)
        print(":: RESP : " + str(resp))

        config = []
        i = 0
        val_req = GetParameters.Request()

        for name, descr in zip(req.names, resp.descriptors):
            param = {}
            param['name'] = name
            param['descr'] = descr
            print("**************NAME : " + str(param['name']))
            print("**************DESCR : " + str(param['descr']))
            if param['descr'].type >= 1 and param['descr'].type <= 4 :
                val_req.names.append(param['name'])
                config.append(param)

        val_resp = self._get_params_client.call(val_req)
        for param, val in zip( config, val_resp.values):
            print("Values    asdfsdf : " + str(val))
            if val.type == 1:
                print("bool_value : " + str(val.bool_value) )
                param['value']=bool(val.bool_value)
            if val.type == 2:
                param['value']=val.integer_value
            if val.type == 3:
                param['value']=val.double_value
            if val.type == 4:
                param['value']=val.string_value
            param['type'] = val.type

        for conf in config:
            print("Config : " + param['name'] + " " +  str(param['descr']) + " " + str(param['type']) )

        return config

    def get_configuration(self):
        return self._node.call().response

    def update_configuration(self, configuration):
        print("Update configuration called : " + str(configuration.items()))
        req = SetParameters.Request()
        for name, vals in configuration.items():
            param = Parameter()
            if vals['type'] == 1:
                print("type  boolean")
                param.name = name
                param.value.type = 1
                param.value.bool_value = vals['value']
                req.parameters.append(param)
            if vals['type'] == 2:
                param.name = name
                param.value.type = 2
                param.value.integer_value = vals['value']
                req.parameters.append(param)
            if vals['type'] == 3:
                param.name = name
                param.value.type = 3
                param.value.double_value = vals['value']
                req.parameters.append(param)
            if vals['type'] == 4:
                param.name = name
                param.value.type = 4
                param.value.string_value = vals['value']
                req.parameters.append(param)

        print("calling with : " + str(req))
        #todo, set parameters in terms of Parameter from service
        return self._set_params_client.call(req)


    def close(self):
        pass
