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

import math
import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, QLocale
from python_qt_binding.QtGui import QDoubleValidator, QIntValidator
from python_qt_binding.QtWidgets import QLabel, QMenu, QWidget
from qt_gui.ros_package_helper import get_package_path
from decimal import Decimal


EDITOR_TYPES = {
    1: 'BooleanEditor',
    2: 'IntegerEditor',
    3: 'DoubleEditor',
    4: 'StringEditor',
}

# These .ui files are frequently loaded multiple times. Since file access
# costs a lot, only load each file once.
package_path = get_package_path('rqt_reconfigure')
ui_bool = os.path.join(package_path, 'share', 'rqt_reconfigure', 'resource',
                       'editor_bool.ui')
ui_str = os.path.join(package_path, 'share', 'rqt_reconfigure', 'resource',
                      'editor_string.ui')
ui_num = os.path.join(package_path, 'share', 'rqt_reconfigure', 'resource',
                      'editor_number.ui')
ui_int = ui_num
ui_enum = os.path.join(package_path, 'share', 'rqt_reconfigure', 'resource',
                       'editor_enum.ui')



class EditorWidget(QWidget):
    '''
    This class is abstract -- its child classes should be instantiated.

    There exist two kinds of "update" methods:
    - _update_paramserver for Parameter Server.
    - update_value for the value displayed on GUI.
    '''

    def __init__(self, updater, config):
        '''
        @param updater: A class that extends threading.Thread.
        @type updater: rqt_reconfigure.param_updater.ParamUpdater
        '''

        super(EditorWidget, self).__init__()

        self._updater = updater
        self.param_name = config['name']
        #self.param_default = config['default']
        self.param_default = config['value']
        self.param_description = config['descr'].description
        self.read_only = config['descr'].read_only
        self.param_type = config['type']

        self.old_value = None

        self.cmenu = QMenu()

    def _update_paramserver(self, value):
        '''
        Update the value on Parameter Server.
        '''
        if value != self.old_value:
            self.update_configuration(value)
            self.old_value = value

    def update_value(self, value):
        '''
        To be implemented in subclass, but still used.

        Update the value that's displayed on the arbitrary GUI component
        based on user's input.

        This method is not called from the GUI thread, so any changes to
        QObjects will need to be done through a signal.
        '''
        self.old_value = value

    def update_configuration(self, value):
        print("Updated : " + str(value))
        #self._updater.update({self.param_name: value})
        self._updater.update({self.param_name: {'value' : value, 'type' : self.param_type}})

    def display(self, grid):
        '''
        Should be overridden in subclass.

        :type grid: QFormLayout
        '''
        self._paramname_label.setText(self.param_name)
#        label_paramname = QLabel(self.param_name)
#        label_paramname.setWordWrap(True)
        self._paramname_label.setMinimumWidth(100)
        grid.addRow(self._paramname_label, self)
        self.setToolTip(self.param_description)
        self._paramname_label.setToolTip(self.param_description)
        self._paramname_label.contextMenuEvent = self.contextMenuEvent

    def close(self):
        '''
        Should be overridden in subclass.
        '''
        pass

    def contextMenuEvent(self, e):
        self.cmenu.exec_(e.globalPos())


class BooleanEditor(EditorWidget):
    _update_signal = Signal(bool)

    def __init__(self, updater, config):
        super(BooleanEditor, self).__init__(updater, config)
        loadUi(ui_bool, self)

        # Initialize to default
        #self.update_value(config['default'])
        print("Value : " + str(config['value']))
        self.update_value(config['value'])

        # Make checkbox update param server
        self._checkbox.stateChanged.connect(self._box_checked)

        # Make param server update checkbox
        self._update_signal.connect(self._checkbox.setChecked)

        if self.read_only:
            self._checbox.setEnabled(False)

    def _box_checked(self, value):
        self._update_paramserver(bool(value))

    def update_value(self, value):
        super(BooleanEditor, self).update_value(value)
        print("update value")
        self._update_signal.emit(value)
        self._checkbox.setChecked(value) #TODO siganl doesn't work (Gonzo)
        print("updated value")



class StringEditor(EditorWidget):
    _update_signal = Signal(str)

    def __init__(self, updater, config):
        super(StringEditor, self).__init__(updater, config)
        loadUi(ui_str, self)

        self._paramval_lineedit.setText(config['value'])

        # Update param server when cursor leaves the text field
        # or enter is pressed.
        self._paramval_lineedit.editingFinished.connect(self.edit_finished)

        # Make param server update text field
        self._update_signal.connect(self._paramval_lineedit.setText)

        # Add special menu items
        self.cmenu.addAction(self.tr('Set to Empty String')).triggered.connect(self._set_to_empty)

        if self.read_only:
            self._paramval_lineedit.setEnabled(False)
            self.cmenu.setEnabled(False)

    def update_value(self, value):
        super(StringEditor, self).update_value(value)
        rospy.logdebug('StringEditor update_value={}'.format(value))
        self._update_signal.emit(value)

    def edit_finished(self):
#        rospy.logdebug('StringEditor edit_finished val={}'.format(
#                                              self._paramval_lineedit.text()))
        self._update_paramserver(self._paramval_lineedit.text())

    def _set_to_empty(self):
        self._update_paramserver('')


class IntegerEditor(EditorWidget):
    _update_signal = Signal(int)

    def __init__(self, updater, config):
        super(IntegerEditor, self).__init__(updater, config)
        loadUi(ui_int, self)

        # Set ranges
        self._min = int(config['descr'].integer_range[0].from_value)
        self._max = int(config['descr'].integer_range[0].to_value)
        self._min_val_label.setText(str(self._min))
        self._max_val_label.setText(str(self._max))

        self._step = int(config['descr'].integer_range[0].step)
        self._slider_horizontal.setSingleStep(self._step)
        self._slider_horizontal.setTickInterval(self._step)
        self._slider_horizontal.setPageStep(self._step)
        self._slider_horizontal.setRange(self._min, self._max)


        # TODO: Fix that the naming of _paramval_lineEdit instance is not
        #       consistent among Editor's subclasses.
        self._paramval_lineEdit.setValidator(QIntValidator(self._min,
                                                           self._max, self))

        # Initialize to default
        self._paramval_lineEdit.setText(str(config['value']))
        self._slider_horizontal.setValue(int(config['value']))

        # Make slider update text (locally)
        self._slider_horizontal.sliderMoved.connect(self._slider_moved)

        # Make keyboard input change slider position and update param server
        self._paramval_lineEdit.editingFinished.connect(self._text_changed)

        # Make slider update param server
        # Turning off tracking means this isn't called during a drag
        self._slider_horizontal.setTracking(False)
        self._slider_horizontal.valueChanged.connect(self._slider_changed)


        # Make the param server update selection
        self._update_signal.connect(self._update_gui)

        # Add special menu items
        self.cmenu.addAction(self.tr('Set to Maximum')).triggered.connect(self._set_to_max)
        self.cmenu.addAction(self.tr('Set to Minimum')).triggered.connect(self._set_to_min)

        if self.read_only:
            self._paramval_lineEdit.setEnabled(False)
            self._slider_horizontal.setEnabled(False)
            self.cmenu.setEnabled(False)

    def _slider_moved(self):
        # This is a "local" edit - only change the text
        print("INTEGER - Slider moved")
        self._paramval_lineEdit.setText(str(
                                self._slider_horizontal.sliderPosition()))

    def _text_changed(self):
        # This is a final change - update param server
        # No need to update slider... update_value() will
        print("INTEGER - Text changed")
        self._slider_horizontal.setValue(int(self._paramval_lineEdit.text()))
        self._update_paramserver(int(self._paramval_lineEdit.text()))

    def _slider_changed(self):
        # This is a final change - update param server
        # No need to update text... update_value() will
        print("INTEGER - Slider changed")
        self._paramval_lineEdit.setText(str(
                                self._slider_horizontal.sliderPosition()))
        self._update_paramserver(self._slider_horizontal.value())

    def update_value(self, value):
        super(IntegerEditor, self).update_value(value)
        self._update_signal.emit(int(value))
        self._update_gui(int(value))

    def _update_gui(self, value):
        # Block all signals so we don't loop
        self._slider_horizontal.blockSignals(True)
        # Update the slider value
        self._slider_horizontal.setValue(value)
        # Make the text match
        self._paramval_lineEdit.setText(str(value))
        self._slider_horizontal.blockSignals(False)

    def _set_to_max(self):
        self.update_value(self._max)
        self._update_paramserver(self._max)

    def _set_to_min(self):
        self.update_value(self._min)
        self._update_paramserver(self._min)



class DoubleEditor(EditorWidget):
    _update_signal = Signal(float)

    def __init__(self, updater, config):
        super(DoubleEditor, self).__init__(updater, config)
        loadUi(ui_num, self)

        #config step
        self._step = float(config['descr'].floating_point_range[0].step)
        print("STEP : " + str(self._step))
        self._slider_horizontal.setSingleStep(self._step)
        self._slider_horizontal.setTickInterval(self._step)
        self._slider_horizontal.setPageStep(self._step)

        # Handle unbounded doubles nicely
        if config['descr'].floating_point_range[0].from_value != -float('inf'):
            self._min = float(config['descr'].floating_point_range[0].from_value)
            self._min_val_label.setText(str(self._min))
        else:
            self._min = -1e10000
            self._min_val_label.setText('-inf')

        if config['descr'].floating_point_range[0].to_value != float('inf'):
            self._max = float(config['descr'].floating_point_range[0].to_value)
            self._max_val_label.setText(str(self._max))
        else:
            self._max = 1e10000
            self._max_val_label.setText('inf')

        if config['descr'].floating_point_range[0].from_value != -float('inf') and config['descr'].floating_point_range[0].to_value != float('inf'):
            self._func = lambda x: x
            self._ifunc = self._func
        else:
            self._func = lambda x: math.atan(x)
            self._ifunc = lambda x: math.tan(x)

        # If we have no range, disable the slider
        self.scale = (self._func(self._max) - self._func(self._min))
        if self.scale <= 0:
            self.scale = 0
            self.setDisabled(True)
        else:
            self.scale = 100 / self.scale

        # Set ranges
        self._slider_horizontal.setRange(self._get_value_slider(self._min),
                                         self._get_value_slider(self._max))
        validator = QDoubleValidator(self._min, self._max, 8, self)
        validator.setLocale(QLocale(QLocale.C))
        self._paramval_lineEdit.setValidator(validator)

        # Initialize to defaults
        self._paramval_lineEdit.setText(str(config['value']))
        self._slider_horizontal.setValue(
                                     self._get_value_slider(config['value']))

        # Make slider update text (locally)
        self._slider_horizontal.sliderMoved.connect(self._slider_moved)

        # Make keyboard input change slider position and update param server
        self._paramval_lineEdit.editingFinished.connect(self._text_changed)

        # Make slider update param server
        # Turning off tracking means this isn't called during a drag
        self._slider_horizontal.setTracking(False)
        self._slider_horizontal.valueChanged.connect(self._slider_changed)

        # Make the param server update selection
        self._update_signal.connect(self._update_gui)

        # Add special menu items
        self.cmenu.addAction(self.tr('Set to Maximum')).triggered.connect(self._set_to_max)
        self.cmenu.addAction(self.tr('Set to Minimum')).triggered.connect(self._set_to_min)

        if self.read_only:
            self._paramval_lineEdit.setEnabled(False)
            self._slider_horizontal.setEnabled(False)
            self.cmenu.setEnabled(False)

    def _slider_moved(self):
        # This is a "local" edit - only change the text
        self._paramval_lineEdit.setText('{0:f}'.format(Decimal(str(
                                                self._get_value_textfield()))))

    def _text_changed(self):
        # This is a final change - update param server
        # No need to update slider... update_value() will
        self._update_gui(float(self._paramval_lineEdit.text()))
        self._update_paramserver(float(self._paramval_lineEdit.text()))

    def _slider_changed(self):
        # This is a final change - update param server
        # No need to update text... update_value() will
        self._paramval_lineEdit.setText('{0:f}'.format(Decimal(str(
                                                    self._get_value_textfield()))))
        self._update_paramserver(self._get_value_textfield())

    def _get_value_textfield(self):
        '''@return: Current value in text field.'''
        return self._ifunc(self._slider_horizontal.sliderPosition() /
                                        self.scale) if self.scale else 0

    def _get_value_slider(self, value):
        '''
        @rtype: double
        '''
        return int(round((self._func(value)) * self.scale))

    def update_value(self, value):
        super(DoubleEditor, self).update_value(value)
        self._update_signal.emit(float(value))

    def _update_gui(self, value):
        # Block all signals so we don't loop
        self._slider_horizontal.blockSignals(True)
        # Update the slider value if not NaN
        if not math.isnan(value):
            self._slider_horizontal.setValue(self._get_value_slider(value))
        elif not math.isnan(self.param_default):
            self._slider_horizontal.setValue(self._get_value_slider(self.param_default))
        # Make the text match
        self._paramval_lineEdit.setText('{0:f}'.format(Decimal(str(value))))
        self._slider_horizontal.blockSignals(False)

    def _set_to_max(self):
        self._update_paramserver(self._max)

    def _set_to_min(self):
        self._update_paramserver(self._min)

    def _set_to_nan(self):
        self._update_paramserver(float('NaN'))


class EnumEditor(EditorWidget):
    _update_signal = Signal(int)

    def __init__(self, updater, config):
        super(EnumEditor, self).__init__(updater, config)

        loadUi(ui_enum, self)

        try:
            enum = eval(config['edit_method'])['enum']
        except:
            rospy.logerr("reconfig EnumEditor) Malformed enum")
            return

        # Setup the enum items
        self.names = [item['name'] for item in enum]
        self.values = [item['value'] for item in enum]

        items = ["%s (%s)" % (self.names[i], self.values[i])
                 for i in range(0, len(self.names))]

        # Add items to the combo box
        self._combobox.addItems(items)

        # Initialize to the default
        self._combobox.setCurrentIndex(self.values.index(config['default']))

        # Make selection update the param server
        self._combobox.currentIndexChanged['int'].connect(self.selected)

        # Make the param server update selection
        self._update_signal.connect(self._update_gui)

        # Bind the context menu
        self._combobox.contextMenuEvent = self.contextMenuEvent

    def selected(self, index):
        self._update_paramserver(self.values[index])

    def update_value(self, value):
        super(EnumEditor, self).update_value(value)
        self._update_signal.emit(self.values.index(value))

    def _update_gui(self, idx):
        # Block all signals so we don't loop
        self._combobox.blockSignals(True)
        self._combobox.setCurrentIndex(idx)
        self._combobox.blockSignals(False)
