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
# Author: Isaac Saito, Ze'ev Klapow

import time

from python_qt_binding.QtCore import QMargins, QSize, Qt, Signal
from python_qt_binding.QtGui import QIcon, QFont
from python_qt_binding.QtWidgets import (QFileDialog, QHBoxLayout,
                                         QPushButton, QWidget, QFormLayout,
                                         QGroupBox, QLabel, QPushButton,
                                         QTabWidget, QVBoxLayout )
from .param_editors import BooleanEditor, DoubleEditor, EditorWidget, \
                           EDITOR_TYPES, EnumEditor, IntegerEditor, \
                           StringEditor
from .param_groups import GroupWidget, find_cfg
from .param_updater import ParamUpdater

import yaml


#class ParamsClientWidget(GroupWidget):
class ParamsClientWidget(QWidget):
    """
    A wrapper of dynamic_reconfigure.client instance.
    Represents a widget where users can view and modify ROS params.
    """

    # public signal
    sig_node_disabled_selected = Signal(str)
    sig_node_state_change = Signal(bool)

    def __init__(self, reconf, node_name):
        """
        :type reconf: dynamic_reconfigure.client
        :type node_name: str
        """

        config = reconf.get_group_descriptions()

        #TODO get Parameters
        # client = node.create_client(
        #     ListParameters,
        #     '{node_name.full_name}/list_parameters'.format_map(locals()))
        # request = ListParameters.Request()
        # response = client.call(request);
        # config.parameters = response.result.names

 #        config['state']
#         config['name']
#         config['parameters']
#         config['groups']

        #rospy.logdebug('DynreconfClientWidget.group_desc=%s', group_desc)
#        super(ParamsClientWidget, self).__init__(ParamUpdater(reconf),
#                                                    group_desc, node_name)
        super(ParamsClientWidget, self).__init__(),

        self.updater = ParamUpdater(reconf)

        # Save and load buttons
        self.button_widget = QWidget(self)
        self.button_header = QHBoxLayout(self.button_widget)
        self.button_header.setContentsMargins(QMargins(0, 0, 0, 0))

        self.load_button = QPushButton()
        self.save_button = QPushButton()

        self.load_button.setIcon(QIcon.fromTheme('document-open'))
        self.save_button.setIcon(QIcon.fromTheme('document-save'))

        self.load_button.clicked[bool].connect(self._handle_load_clicked)
        self.save_button.clicked[bool].connect(self._handle_save_clicked)

        self.button_header.addWidget(self.save_button)
        self.button_header.addWidget(self.load_button)

        self.setMinimumWidth(150)

        self.reconf = reconf
        self.updater.start()
        self.reconf.config_callback = self.config_callback
        self._node_grn = node_name

        verticalLayout = QVBoxLayout(self)
        verticalLayout.setContentsMargins(QMargins(0, 0, 0, 0))

        _widget_nodeheader = QWidget()
        _h_layout_nodeheader = QHBoxLayout(_widget_nodeheader)
        _h_layout_nodeheader.setContentsMargins(QMargins(0, 0, 0, 0))

        self.nodename_qlabel = QLabel(self)
        font = QFont('Trebuchet MS, Bold')
        font.setUnderline(True)
        font.setBold(True)

        # Button to close a node.
        _icon_disable_node = QIcon.fromTheme('window-close')
        _bt_disable_node = QPushButton(_icon_disable_node, '', self)
        _bt_disable_node.setToolTip('Hide this node')
        _bt_disable_node_size = QSize(36, 24)
        _bt_disable_node.setFixedSize(_bt_disable_node_size)
        _bt_disable_node.pressed.connect(self._node_disable_bt_clicked)

        _h_layout_nodeheader.addWidget(self.nodename_qlabel)
        _h_layout_nodeheader.addWidget(_bt_disable_node)

        self.nodename_qlabel.setAlignment(Qt.AlignCenter)
        font.setPointSize(10)
        self.nodename_qlabel.setFont(font)
        grid_widget = QWidget(self)
        self.grid = QFormLayout(grid_widget)
        verticalLayout.addWidget(_widget_nodeheader)
        verticalLayout.addWidget(grid_widget, 1)
        # Again, these UI operation above needs to happen in .ui file.

        self.tab_bar = None  # Every group can have one tab bar
        self.tab_bar_shown = False

        self.editor_widgets = []
        self._param_names = []

        self._create_node_widgets(config)

#        rospy.logdebug('Groups node name={}'.format(nodename))
        self.nodename_qlabel.setText(node_name)

    def _create_node_widgets(self, config):
        '''
        :type config: Dict?
        '''
        i_debug = 0
#        for param in config['parameters']:

        for param in config:
            begin = time.time() * 1000
            editor_type = '(none)'

            #if param['edit_method']:  #TODO this should go away (Gonzo)
            #    widget = EnumEditor(self.updater, param)
            if param.type in EDITOR_TYPES:
#                rospy.logdebug('GroupWidget i_debug=%d param type =%s',
#                               i_debug,
#                               param['type'])
                editor_type = EDITOR_TYPES[param.type]
                widget = eval(editor_type)(self.updater, param)

            self.editor_widgets.append(widget)
            self._param_names.append(param.name)

#            rospy.logdebug('groups._create_node_widgets num editors=%d',
#                           i_debug)

            end = time.time() * 1000
            time_elap = end - begin
#            rospy.logdebug('ParamG editor={} loop=#{} Time={}msec'.format(
#                                              editor_type, i_debug, time_elap))
            i_debug += 1

        for i, ed in enumerate(self.editor_widgets):
            ed.display(self.grid)

#        rospy.logdebug('GroupWdgt._create_node_widgets len(editor_widgets)=%d',
#                       len(self.editor_widgets))

    def display(self, grid):
        grid.addRow(self)

    def get_node_grn(self):

        return self._node_grn

    def config_callback(self, config):

        #TODO: Think about replacing callback architecture with signals.

        if config:
            # TODO: should use config.keys but this method doesnt exist

            names = [name for name, v in config.items()]
            # v isn't used but necessary to get key and put it into dict.
        #    rospy.logdebug('config_callback name={} v={}'.format(name, v))

            for widget in self.editor_widgets:
                if isinstance(widget, EditorWidget):
                    if widget.param_name in names:
                        rospy.logdebug('EDITOR widget.param_name=%s',
                                       widget.param_name)
                        widget.update_value(config[widget.param_name])
                elif isinstance(widget, GroupWidget):
                    cfg = find_cfg(config, widget.param_name)
#                    rospy.logdebug('GROUP widget.param_name=%s',
#                                   widget.param_name)
                    widget.update_group(cfg)

    def _handle_load_clicked(self):
        filename = QFileDialog.getOpenFileName(
                self, self.tr('Load from File'), '.',
                self.tr('YAML file {.yaml} (*.yaml)'))
        if filename[0] != '':
            self.load_param(filename[0])

    def _handle_save_clicked(self):
        filename = QFileDialog.getSaveFileName(
                self, self.tr('Save parameters to file...'), '.',
                self.tr('YAML files {.yaml} (*.yaml)'))
        if filename[0] != '':
            self.save_param(filename[0])

    def save_param(self, filename):
        configuration = self.reconf.get_configuration()
        if configuration is not None:
            with file(filename, 'w') as f:
                yaml.dump(configuration, f)

    def load_param(self, filename):
        with file(filename, 'r') as f:
            configuration = {}
            for doc in yaml.load_all(f.read()):
                configuration.update(doc)

        try:
            self.reconf.update_configuration(configuration)
        except ServiceException as e:
        #    rospy.logwarn('Call for reconfiguration wasn\'t successful because: %s', e.message)
            pass
        except DynamicReconfigureParameterException as e:
        #    rospy.logwarn('Reconfiguration wasn\'t successful because: %s', e.message)
            pass
        except DynamicReconfigureCallbackException as e:
        #    rospy.logwarn('Reconfiguration wasn\'t successful because: %s', e.message)
            pass

    def _node_disable_bt_clicked(self):
#        rospy.logdebug('param_gs _node_disable_bt_clicked')
        self.sig_node_disabled_selected.emit(self._toplevel_treenode_name)

    def close(self):
        self.reconf.close()
        self.updater.stop()

        for w in self.editor_widgets:
            w.close()

        self.deleteLater()

    def filter_param(self, filter_key):
        #TODO impl
        pass
