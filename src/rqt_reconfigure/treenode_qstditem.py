# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
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
# Author: Isaac Saito

from __future__ import division

import threading
import time

import dynamic_reconfigure.client

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QBrush, QStandardItem

from rospy.exceptions import ROSException

from rqt_py_common.data_items import ReadonlyItem

from rqt_reconfigure import logging
from rqt_reconfigure.param_client_widget import ParamClientWidget


class ParamserverConnectThread(threading.Thread):

    def __init__(self, parent, param_name_raw):
        super(ParamserverConnectThread, self).__init__()
        self._parent = parent
        self._raw_param_name = param_name_raw

    def run(self):
        param_client = None
        try:
            param_client = dynamic_reconfigure.client.Client(
                str(self._raw_param_name), timeout=5.0)
            logging.debug(
                'ParamserverConnectThread param_client={}'.format(
                    param_client
                ))
            self._parent.set_param_client(param_client)
        except ROSException as e:
            raise type(e)(
                e.message + "TreenodeQstdItem. Couldn't connect to {}".format(
                    self._raw_param_name
                ))


class TreenodeQstdItem(ReadonlyItem):
    """
    Extending ReadonlyItem - the display content of this item shouldn't be
    modified.
    """

    NODE_FULLPATH = 1

    def __init__(self, *args):
        """
        :param args[0]: str (will become 1st arg of QStandardItem)
        :param args[1]: integer value that indicates whether this class
                               is node that has GRN (Graph Resource Names, see
                               http://www.ros.org/wiki/Names). This can be None
        """
        grn_current_treenode = args[0]
        self._raw_param_name = grn_current_treenode
        self._set_param_name(grn_current_treenode)
        super(TreenodeQstdItem, self).__init__(grn_current_treenode)

        # dynamic_reconfigure.client.Client
        self._param_client = None
        # ParamClientWidget
        self._param_client_widget = None

        self._is_rosnode = False

        self._lock = threading.Lock()
        self._paramserver_connect_thread = None

        try:
            if args[1]:
                self._is_rosnode = True
        except IndexError:  # tuple index out of range etc.
            logging.error('TreenodeQstdItem IndexError')

    def set_param_client(self, param_client):
        """
        @param param_client: dynamic_reconfigure.client.Client
        """
        self._param_client = param_client
        logging.debug('Qitem set param_client={} param={}'.format(
            self._param_client, self._raw_param_name
        ))

    def clear_param_client(self):
        if self._param_client is not None:
            self._param_client.close()
            del self._param_client
            self._param_client = None

    def get_param_client_widget(self):
        """
        @rtype: ParamClientWidget (QWidget)
        @return: None if param_client is not yet generated.
        @raise ROSException:
        """
        if not self._param_client_widget:
            logging.debug('get param_client={}'.format(
                self._param_client
            ))
            logging.debug('In get_param_client_widget 1')
            if not self._param_client:
                self.connect_param_server()
            logging.debug('In get_param_client_widget 2')

            timeout = 3 * 100
            loop = 0
            # Loop until _param_client is set. self._param_client gets
            # set from different thread (in ParamserverConnectThread).
            while self._param_client is None:
                # Avoid deadlock
                if timeout < loop:
                    # Make itself unclickable
                    self.setEnabled(False)
                    raise ROSException('param client failed')

                time.sleep(0.01)
                loop += 1
                logging.debug('In get_param_client_widget loop#{}'.format(loop))

            logging.debug('In get_param_client_widget 4')
            self._param_client_widget = ParamClientWidget(
                self._param_client, self._raw_param_name
            )
            # Creating the ParamClientWidget transfers ownership of the
            # _param_client to it. If it is destroyed from Qt, we need to
            # clear our reference to it and stop the param server thread we
            # had.
            self._param_client_widget.destroyed.connect(
                self.clear_param_client_widget)
            self._param_client_widget.destroyed.connect(
                self.disconnect_param_server)
            logging.debug('In get_param_client_widget 5')

        else:
            pass
        return self._param_client_widget

    def clear_param_client_widget(self):
        self._param_client_widget = None

    def connect_param_server(self):
        """
        Connect to parameter server using dynamic_reconfigure client.
        Behavior is delegated to a private method _connect_param_server, and
        its return value, client, is set to member variable.

        @return void
        @raise ROSException:
        """
        # If the treenode doesn't represent ROS Node, return None.
        with self._lock:
            if not self._is_rosnode:
                logging.error('connect_param_server failed due to missing '
                              'ROS Node. Return with nothing.')
                return

            if not self._param_client:
                if self._paramserver_connect_thread:
                    if self._paramserver_connect_thread.isAlive():
                        self._paramserver_connect_thread.join(1)
                self._paramserver_connect_thread = ParamserverConnectThread(
                    self, self._raw_param_name)
                self._paramserver_connect_thread.start()

    def disconnect_param_server(self):
        with self._lock:
            if self._paramserver_connect_thread:
                # Try to stop the thread
                if self._paramserver_connect_thread.isAlive():
                    self._paramserver_connect_thread.join(1)
                del self._paramserver_connect_thread
                self._paramserver_connect_thread = None
            self.clear_param_client()

    def enable_param_items(self):
        """
        Create QStdItem per parameter and addColumn them to myself.
        :rtype: None if _param_client is not initiated.
        """
        if not self._param_client_widget:
            return None
        param_names = self._param_client_widget.get_treenode_names()
        param_names_items = []
        brush = QBrush(Qt.lightGray)
        for param_name in param_names:
            item = ReadonlyItem(param_name)
            item.setBackground(brush)
            param_names_items.append(item)
        logging.debug('enable_param_items len of param_names={}'.format(
            len(param_names_items)
        ))
        self.appendColumn(param_names_items)

    def _set_param_name(self, param_name):
        """
        :param param_name: A string formatted as GRN (Graph Resource Names, see
                           http://www.ros.org/wiki/Names).
                           Example: /paramname/subpara/subsubpara/...
        """
        logging.debug('_set_param_name param_name={} '.format(param_name))

        #  separate param_name by forward slash
        self._list_treenode_names = param_name.split('/')

        #  Deleting the 1st elem which is zero-length str.
        del self._list_treenode_names[0]

        self._toplevel_treenode_name = self._list_treenode_names[0]

        logging.debug('param_name={} node_name={} _list_params[-1]={}'.format(
            param_name, self._toplevel_treenode_name,
            self._list_treenode_names[-1]
        ))

    def get_param_name_toplv(self):
        """
        :rtype: String of the top level param name.
        """
        return self._name_top

    def get_raw_param_name(self):
        return self._raw_param_name

    def get_treenode_names(self):
        """
        :rtype: List of string. Null if param
        """
        # TODO: what if self._list_treenode_names is empty or null?
        return self._list_treenode_names

    def get_node_name(self):
        """
        :return: A value of single tree node (ie. NOT the fullpath node name).
                 Ex. suppose fullpath name is /top/sub/subsub/subsubsub and you
                     are at 2nd from top, the return value is subsub.
        """
        return self._toplevel_treenode_name

    def type(self):
        return QStandardItem.UserType
