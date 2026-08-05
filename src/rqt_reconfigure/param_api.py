# Copyright (c) 2019 Open Source Robotics Foundation, Inc.
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
#    * Neither the name of the copyright holder nor the names of its
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

from threading import Event

from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.srv import DescribeParameters
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import ListParameters
from rcl_interfaces.srv import SetParameters

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_parameter_events

from rqt_reconfigure import logging

# How long to wait for the remote node's parameter services to show up in the
# ROS graph. Only paid when the node is not up (yet) at all.
_DISCOVERY_TIMEOUT = 5.0
# Timeout for a single attempt at an individual service call. A healthy,
# spinning node answers well within this window.
_CALL_TIMEOUT = 1.0
# Number of attempts per service call. Retrying works around the race where
# service_is_ready() already reports True from the ROS graph cache while the
# DDS request writer has not finished matching the server's request reader,
# which silently drops the request.
_CALL_ATTEMPTS = 3


class AsyncServiceCallFailed(Exception):

    def __init__(self, message='asynchronous service call failed', hint=''):
        self.message = message if not hint else message + ': ' + hint
        super().__init__(self.message)


class ParamClient(object):

    def __init__(self, node, remote_node_name, param_change_callback=None):

        self._node = node
        self._remote_node_name = remote_node_name
        # Keep these entities out of the node's default callback group. That
        # group is a MutuallyExclusiveCallbackGroup shared by every rqt plugin
        # loaded in this process and by every other node panel opened here.
        # While any entity in it executes, the executor's wait set excludes all
        # the others (rclpy Executor._wait_for_ready_callbacks() filters on
        # can_execute()), so our service responses would be delayed past the
        # timeout in _call_service() below.
        #
        # Reentrancy is safe here: _on_parameter_event() only reads
        # _remote_node_name and builds local lists, and the clients' response
        # callbacks only fulfil their own future. Reconsider if this class ever
        # gains mutable state shared between callbacks.
        self._callback_group = ReentrantCallbackGroup()
        self._get_params_client = self._node.create_client(
            GetParameters, '{remote_node_name}/get_parameters'.format_map(locals()),
            callback_group=self._callback_group
        )
        self._set_params_client = self._node.create_client(
            SetParameters, '{remote_node_name}/set_parameters'.format_map(locals()),
            callback_group=self._callback_group
        )
        self._list_params_client = self._node.create_client(
            ListParameters, '{remote_node_name}/list_parameters'.format_map(locals()),
            callback_group=self._callback_group
        )
        self._describe_params_client = self._node.create_client(
            DescribeParameters, '{remote_node_name}/describe_parameters'.format_map(locals()),
            callback_group=self._callback_group
        )
        self._param_events_subscription = self._node.create_subscription(
            ParameterEvent, '/parameter_events', self._on_parameter_event,
            qos_profile_parameter_events, callback_group=self._callback_group
        )
        self._param_change_callback = param_change_callback
        # Parameters this node advertises but will not return a value for, so
        # that get_parameters() reports each of them only once.
        self._unreadable_params = set()

    def _on_parameter_event(self, event):
        if event.node != self._remote_node_name:
            return
        if self._param_change_callback is not None:
            try:
                self._param_change_callback(
                    [Parameter.from_parameter_msg(p) for p in event.new_parameters],
                    [Parameter.from_parameter_msg(p) for p in event.changed_parameters],
                    [Parameter.from_parameter_msg(p) for p in event.deleted_parameters]
                )
            except Exception as e:
                # This runs on the executor thread of the node shared by all rqt
                # plugins. rqt_gui_py's RclpySpinner does not guard its spin
                # loop, and MultiThreadedExecutor re-raises task exceptions
                # there, so anything escaping this callback would stop that node
                # from spinning for every plugin for the rest of the process.
                # See ros-visualization/rqt_reconfigure#146.
                logging.warn(
                    'Failed to handle parameter event for node'
                    ' {}: {}'.format(self._remote_node_name, e)
                )

    def list_parameters(self):
        list_params_request = ListParameters.Request()
        list_params_response = self._call_service(self._list_params_client, list_params_request)
        return list_params_response.result.names

    def get_parameters(self, names):
        """
        Read the given parameters, skipping the ones the node refuses to serve.

        The returned list may be shorter than ``names``, so callers must not
        assume it lines up positionally with anything derived from ``names``.
        """
        if not names:
            return []

        pairs = self._get_parameters_partial(list(names))

        unreadable = [name for name in names if name not in {n for n, _ in pairs}]
        # Only report each parameter once: this runs again on every keystroke in
        # the parameter filter box.
        new_unreadable = [n for n in unreadable if n not in self._unreadable_params]
        if new_unreadable:
            self._unreadable_params.update(new_unreadable)
            logging.warn(
                'Node {} did not return a value for {} parameter(s), which are most'
                ' likely declared without being initialized: {}'.format(
                    self._remote_node_name, len(new_unreadable), ', '.join(new_unreadable)
                )
            )

        return [
            Parameter.from_parameter_msg(ParameterMsg(name=name, value=value))
            for name, value in pairs
        ]

    def _get_parameters_partial(self, names):
        """
        Return the (name, value) pairs that could be read, as a list.

        rclcpp's GetParameters handler is all or nothing: if any requested
        parameter was declared without being initialized, it logs a warning and
        answers with an empty value list instead of a partial one. A single such
        parameter would therefore hide every parameter of the node. Bisect the
        request so that only the offending parameters are dropped, which costs
        far fewer round trips than asking for every parameter separately.

        A request that times out still raises, so an unresponsive node fails
        fast instead of being retried once per parameter.
        """
        get_params_request = GetParameters.Request()
        get_params_request.names = names
        values = self._call_service(self._get_params_client, get_params_request).values

        if len(values) == len(names):
            return list(zip(names, values))
        if len(names) == 1:
            return []

        middle = len(names) // 2
        return (self._get_parameters_partial(names[:middle]) +
                self._get_parameters_partial(names[middle:]))

    def describe_parameters(self, names):
        describe_params_request = DescribeParameters.Request()
        describe_params_request.names = names
        describe_params_response = self._call_service(self._describe_params_client,
                                                      describe_params_request)
        return describe_params_response.descriptors

    def set_parameters(self, parameters):
        set_params_request = SetParameters.Request()
        set_params_request.parameters = [p.to_parameter_msg() for p in parameters]
        # Unlike the read-only calls above this one is not guaranteed to be
        # idempotent, because the remote node's on-set-parameters callback may
        # have side effects. Allow a single retry, which is enough to survive the
        # matching race described at _CALL_ATTEMPTS on this service's own
        # request writer, while bounding the risk of applying a change twice.
        return self._call_service(self._set_params_client, set_params_request, attempts=2)

    def close(self):
        self._node.destroy_subscription(self._param_events_subscription)
        self._node.destroy_client(self._describe_params_client)
        self._node.destroy_client(self._list_params_client)
        self._node.destroy_client(self._set_params_client)
        self._node.destroy_client(self._get_params_client)

    def _call_service(self, client, request, timeout=_CALL_TIMEOUT, attempts=_CALL_ATTEMPTS):
        if not client.service_is_ready():
            if not client.wait_for_service(_DISCOVERY_TIMEOUT):
                raise AsyncServiceCallFailed(hint='timed out waiting for service')

        # It is possible that a node has the parameter services but is not
        # spinning. In that is the case, every attempt below times out. It is
        # also possible that only the first attempt is lost because the request
        # writer had not finished matching yet, hence the retries.
        for _attempt in range(attempts):
            event = Event()
            future = client.call_async(request)
            # Bind the event explicitly so a done callback left over from an
            # earlier attempt can never signal a later attempt's event.
            future.add_done_callback(lambda _, event=event: event.set())

            # future.done() covers the response landing in the race between the
            # wait above expiring and this check.
            if event.wait(timeout) or future.done():
                return future.result()

            # Abandon this attempt's request so it does not stay in the client's
            # pending requests forever. cancel() synchronously invokes the
            # remove_pending_request done callback registered by call_async().
            future.cancel()

        raise AsyncServiceCallFailed(hint='the target node may not be spinning')


def create_param_client(node, remote_node_name, param_change_callback=None):
    return ParamClient(node, remote_node_name, param_change_callback)


def _has_parameters(node, node_name, node_namespace):
    # Get all of the services provided by a node (node_name)
    for service_name, service_types in node.get_service_names_and_types_by_node(
            node_name, node_namespace):

        # Make sure the node supports the ListParameters service
        if 'rcl_interfaces/srv/ListParameters' in service_types:
            return True
    return False


def find_nodes_with_params(node):
    names_and_namespaces = node.get_node_names_and_namespaces()
    node_list = []
    for node_name, node_namespace in names_and_namespaces:
        if _has_parameters(node, node_name, node_namespace):
            full_name = node_namespace.rstrip('/') + '/' + node_name
            node_list.append(full_name)
    return node_list
