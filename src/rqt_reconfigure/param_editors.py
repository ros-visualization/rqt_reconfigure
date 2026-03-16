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
# Author: Isaac Saito, Ze'ev Klapow

import array
from decimal import Decimal
import json
import math
import os
import sys

from ament_index_python import get_resource

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QEvent, QLocale, Signal
from python_qt_binding.QtGui import QDoubleValidator, QIntValidator
from python_qt_binding.QtWidgets import QMenu, QWidget

from rclpy.parameter import Parameter

from rqt_reconfigure import logging

# These .ui files are frequently loaded multiple times. Since file access
# costs a lot, only load each file once.
_, package_path = get_resource('packages', 'rqt_reconfigure')


class EditorWidget(QWidget):
    """
    This class is abstract -- its child classes should be instantiated.

    There exist two kinds of "update" methods:
    - _update_paramserver for Parameter Server.
    - update_value for the value displayed on GUI.
    """

    def __init__(self, param_client, parameter, descriptor):
        super(EditorWidget, self).__init__()

        self._param_client = param_client
        self.parameter = parameter
        self.descriptor = descriptor

        self.cmenu = QMenu()

    def update_remote(self, value):
        # Update the value on Parameter Server.
        try:
            self._param_client.set_parameters([self.parameter])
        except Exception as e:
            logging.warn('Failed to set parameters for node: ' + str(e))

    def update_local(self, value):
        """
        To be implemented in subclass, but still used.

        Update the value that's displayed on the arbitrary GUI component
        based on user's input.

        This method is not called from the GUI thread, so any changes to
        QObjects will need to be done through a signal.
        """
        self.parameter = Parameter(
            name=self.parameter.name,
            type_=self.parameter.type_,
            value=value
        )

    def update(self, value):
        old_value = self.parameter.value
        self.update_local(value)
        if self.parameter.value != old_value:
            self.update_remote(value)

    def display(self, grid):
        """
        Must be overridden in subclass.

        :type grid: QFormLayout
        """
        self._paramname_label.setText(self.parameter.name)
        self._paramname_label.setMinimumWidth(100)
        grid.addRow(self._paramname_label, self)
        self.setToolTip(self.descriptor.description)
        self._paramname_label.setToolTip(self.descriptor.description)
        self._paramname_label.contextMenuEvent = self.contextMenuEvent

    def hide(self, grid):
        grid.removeRow(self)

    def close(self):
        # Should be overridden in subclass.
        pass

    def contextMenuEvent(self, e):
        self.cmenu.exec_(e.globalPos())


class BooleanEditor(EditorWidget):
    _update_signal = Signal(bool)

    def __init__(self, *args, **kwargs):
        super(BooleanEditor, self).__init__(*args, **kwargs)
        ui_bool = os.path.join(
            package_path, 'share', 'rqt_reconfigure', 'resource',
            'editor_bool.ui')
        loadUi(ui_bool, self)

        # Set inital value
        self._checkbox.setChecked(self.parameter.value)

        # Make checkbox update param server
        self._checkbox.stateChanged.connect(self._box_checked)

        self._update_signal.connect(self._checkbox.setChecked)

        if self.descriptor.read_only:
            self._checkbox.setEnabled(False)

    def _box_checked(self, value):
        self.update(bool(value))

    def update_local(self, value):
        super(BooleanEditor, self).update_local(value)
        self._update_signal.emit(value)


class StringEditor(EditorWidget):
    _update_signal = Signal(str)

    def __init__(self, *args, **kwargs):
        super(StringEditor, self).__init__(*args, **kwargs)
        ui_str = os.path.join(
            package_path, 'share', 'rqt_reconfigure', 'resource',
            'editor_string.ui')
        loadUi(ui_str, self)

        self._paramval_lineedit.setText(self.parameter.value)

        # Update param server when cursor leaves the text field
        # or enter is pressed.
        self._paramval_lineedit.editingFinished.connect(self.edit_finished)

        # Make param server update text field
        self._update_signal.connect(self._paramval_lineedit.setText)

        # Add special menu items
        self.cmenu.addAction(self.tr('Set to Empty String')
                             ).triggered.connect(self._set_to_empty)

        if self.descriptor.read_only:
            self._paramval_lineedit.setReadOnly(True)
            self.cmenu.setEnabled(False)

    def update_local(self, value):
        super(StringEditor, self).update_local(value)
        logging.debug('StringEditor update_local={}'.format(value))
        self._update_signal.emit(value)

    def edit_finished(self):
        logging.debug('StringEditor edit_finished val={}'.format(
            self._paramval_lineedit.text()))
        self.update(self._paramval_lineedit.text())

    def _set_to_empty(self):
        self.update('')


class IntegerEditor(EditorWidget):

    def __init__(self, *args, **kwargs):
        super(IntegerEditor, self).__init__(*args, **kwargs)
        ui_int = os.path.join(
            package_path, 'share', 'rqt_reconfigure', 'resource',
            'editor_number.ui')
        loadUi(ui_int, self)

        if(len(self.descriptor.integer_range) > 0):
            # Set ranges
            self._min = int(self.descriptor.integer_range[0].from_value)
            self._max = int(self.descriptor.integer_range[0].to_value)
            self._min_val_label.setText(str(self._min))
            self._max_val_label.setText(str(self._max))

            if self._max > 2**31-1 or self._min < -2**31:
                self.scale = (2**31-1) / (self._max - self._min)
                logging.warn(
                    f'The range of this parameter ({self._min} to {self._max}) is too large '
                    f'for the slider to handle. '
                    f'Scaling down to fit within 32 bits with factor {self.scale}.'
                )
            else:
                self.scale = 1
                # TODO: Fix that the naming of _paramval_lineEdit instance is not
                #       consistent among Editor's subclasses.
                self._paramval_lineEdit.setValidator(QIntValidator(self._min, self._max, self))

            self._step = int(self.descriptor.integer_range[0].step)
            self._slider_horizontal.setSingleStep(self._get_value_slider(self._step))
            self._slider_horizontal.setTickInterval(self._get_value_slider(self._step))
            self._slider_horizontal.setPageStep(self._get_value_slider(self._step))
            self._slider_horizontal.setRange(self._get_value_slider(self._min),
                                             self._get_value_slider(self._max))

            self._slider_horizontal.setValue(self._get_value_slider(int(self.parameter.value)))

            # Make slider update text (locally)
            self._slider_horizontal.sliderMoved.connect(self._slider_moved)

            # Make slider update param server
            # Turning off tracking means this isn't called during a drag
            self._slider_horizontal.setTracking(False)
            self._slider_horizontal.valueChanged.connect(self._slider_changed)

            # Add special menu items
            self.cmenu.addAction(self.tr('Set to Maximum')
                                 ).triggered.connect(self._set_to_max)
            self.cmenu.addAction(self.tr('Set to Minimum')
                                 ).triggered.connect(self._set_to_min)

        else:
            self._paramval_lineEdit.setValidator(QIntValidator())
            self._min_val_label.setVisible(False)
            self._max_val_label.setVisible(False)
            self._slider_horizontal.setVisible(False)
            self.scale = 0

        # Make keyboard input change slider position and update param server
        self._paramval_lineEdit.editingFinished.connect(self._text_changed)

        # Initialize to default
        self._paramval_lineEdit.setText(str(self.parameter.value))

        if self.descriptor.read_only:
            self._paramval_lineEdit.setEnabled(False)
            self._slider_horizontal.setEnabled(False)
            self.cmenu.setEnabled(False)

        # Don't process wheel events when not focused
        self._slider_horizontal.installEventFilter(self)

    def _get_value_slider(self, value):
        return int(round((value) * self.scale))

    def _get_value_textfield(self):
        return self._slider_horizontal.sliderPosition() / self.scale if self.scale else 0

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Wheel and not obj.hasFocus():
            return True
        return super(EditorWidget, self).eventFilter(obj, event)

    def _clamp_int(self, value):
        return max(-2**31, min(2**31 - 1, value))

    def _slider_moved(self):
        # This is a "local" edit - only change the text
        self._paramval_lineEdit.setText(str(
            self._get_value_textfield()))

    def _text_changed(self):
        # This is a final change - update param server
        # No need to update slider... update() will
        logging.debug('_text_changed called with text: {}'.format(self._paramval_lineEdit.text()))
        self.update(int(self._paramval_lineEdit.text()))

    def _slider_changed(self):
        # This is a final change - update param server
        # No need to update text... update() will
        logging.debug('_slider_changed called with value: {}'.format(self._get_value_textfield()))
        self.update(int(self._get_value_textfield()))

    def update_local(self, value):
        logging.debug('update_local called with value: {}'.format(value))
        super(IntegerEditor, self).update_local(value)
        self._update_gui(int(value))

    def _update_gui(self, value):
        logging.debug('_update_gui called with value: {}'.format(value))
        # Block all signals so we don't loop
        self._slider_horizontal.blockSignals(True)
        # Update the slider value
        self._slider_horizontal.setValue(self._get_value_slider(value))
        # Make the text match
        self._paramval_lineEdit.setText(str(value))
        self._slider_horizontal.blockSignals(False)

    def _set_to_max(self):
        self.update(self._max)

    def _set_to_min(self):
        self.update(self._min)


class DoubleEditor(EditorWidget):
    _update_signal = Signal(float)

    def __init__(self, *args, **kwargs):
        super(DoubleEditor, self).__init__(*args, **kwargs)
        ui_num = os.path.join(
            package_path, 'share', 'rqt_reconfigure', 'resource',
            'editor_number.ui'
        )
        loadUi(ui_num, self)
        if len(self.descriptor.floating_point_range) > 0:
            # Handle unbounded doubles nicely
            self._min = float(self.descriptor.floating_point_range[0].from_value)
            self._min_val_label.setText(str(self._min))

            self._max = float(self.descriptor.floating_point_range[0].to_value)
            self._max_val_label.setText(str(self._max))

            self._func = lambda x: x
            self._ifunc = self._func

            # If we have no range, slider is disabled automatically
            self.scale = (self._func(self._max) - self._func(self._min))
            if math.isfinite(self.scale):
                self.scale = 100 / self.scale
            else:
                self.scale = 100 / (sys.float_info.max - sys.float_info.min)

            # config step
            self._step = float(self.descriptor.floating_point_range[0].step)
            self._slider_horizontal.setSingleStep(self._get_value_slider(self._step))
            self._slider_horizontal.setTickInterval(self._get_value_slider(self._step))
            self._slider_horizontal.setPageStep(self._get_value_slider(self._step))

            # Set ranges
            self._slider_horizontal.setRange(self._get_value_slider(self._min),
                                             self._get_value_slider(self._max))
            validator = QDoubleValidator(self._min, self._max, 8, self)
            validator.setLocale(QLocale(QLocale.C))
            self._paramval_lineEdit.setValidator(validator)

            self._slider_horizontal.setValue(
                self._get_value_slider(self.parameter.value)
            )

            # Make slider update text (locally)
            self._slider_horizontal.sliderMoved.connect(self._slider_moved)

            # Make slider update param server
            # Turning off tracking means this isn't called during a drag
            self._slider_horizontal.setTracking(False)
            self._slider_horizontal.valueChanged.connect(self._slider_changed)
        else:
            self._paramval_lineEdit.setValidator(QDoubleValidator())
            self._min_val_label.setVisible(False)
            self._max_val_label.setVisible(False)
            self._slider_horizontal.setVisible(False)
            self._func = lambda x: math.atan(x)
            self._ifunc = lambda x: math.tan(x)
            self.scale = 0

        # Initialize to defaults
        self._paramval_lineEdit.setText(str(self.parameter.value))

        # Make keyboard input change slider position and update param server
        self._paramval_lineEdit.editingFinished.connect(self._text_changed)

        # Make the param server update selection
        self._update_signal.connect(self._update_gui)

        # Add special menu items
        self.cmenu.addAction(self.tr('Set to Maximum')
                             ).triggered.connect(self._set_to_max)
        self.cmenu.addAction(self.tr('Set to Minimum')
                             ).triggered.connect(self._set_to_min)

        if self.descriptor.read_only:
            self._paramval_lineEdit.setEnabled(False)
            self._slider_horizontal.setEnabled(False)
            self.cmenu.setEnabled(False)

        # Don't process wheel events when not focused
        self._slider_horizontal.installEventFilter(self)

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Wheel and not obj.hasFocus():
            return True
        return super(EditorWidget, self).eventFilter(obj, event)

    def _slider_moved(self):
        # This is a "local" edit - only change the text
        self._paramval_lineEdit.setText('{0:f}'.format(Decimal(str(
            self._get_value_textfield()))))

    def _text_changed(self):
        # This is a final change - update param server
        # No need to update slider... update() will
        self.update(float(self._paramval_lineEdit.text()))

    def _slider_changed(self):
        # This is a final change - update param server
        # No need to update text... update() will
        self.update(self._get_value_textfield())

    def _get_value_textfield(self):
        return self._ifunc(
            self._slider_horizontal.sliderPosition() / self.scale
        ) if self.scale else 0

    def _get_value_slider(self, value):

        if math.isfinite(value):
            return int(round((self._func(value)) * self.scale))
        if math.isinf(value):
            return int(round((self._func(math.copysign(100, value)))))
        # nan
        return 0

    def update_local(self, value):
        super(DoubleEditor, self).update_local(value)
        self._update_gui(value)
        self._update_signal.emit(value)

    def _update_gui(self, value):
        # Block all signals so we don't loop
        self._slider_horizontal.blockSignals(True)
        # Update the slider value if not NaN
        if not math.isnan(value):
            self._slider_horizontal.setValue(self._get_value_slider(value))
        elif not math.isnan(self.param_default):
            self._slider_horizontal.setValue(
                self._get_value_slider(self.param_default))
        # Make the text match
        self._paramval_lineEdit.setText('{0:f}'.format(Decimal(str(value))))
        self._slider_horizontal.blockSignals(False)

    def _set_to_max(self):
        self.update(self._max)

    def _set_to_min(self):
        self.update(self._min)

    def _set_to_nan(self):
        self.update(float('NaN'))


class ArrayEditor(EditorWidget):
    _update_signal = Signal(list)

    def __init__(self, *args, **kwargs):
        super(ArrayEditor, self).__init__(*args, **kwargs)
        ui_str = os.path.join(
            package_path, 'share', 'rqt_reconfigure', 'resource',
            'editor_string.ui')
        loadUi(ui_str, self)

        if isinstance(self.parameter.value, array.array):
            self._paramval_lineedit.setText(str(self.parameter.value.tolist()))
        else:
            self._paramval_lineedit.setText(str(self.parameter.value))

        # Update param server when cursor leaves the text field
        # or enter is pressed.
        self._paramval_lineedit.editingFinished.connect(self.edit_finished)

        # Make param server update text field
        self._update_signal.connect(self._update_gui)

        # Add special menu items
        self.cmenu.addAction(self.tr('Set to Empty String')
                             ).triggered.connect(self._set_to_empty)

        if self.descriptor.read_only:
            self._paramval_lineedit.setReadOnly(True)
            self.cmenu.setEnabled(False)

    def update_local(self, value):
        super(ArrayEditor, self).update_local(value)
        logging.debug('DoubleArrayEditor update_local={}'.format(value))
        if isinstance(self.parameter.value, array.array):
            self._update_signal.emit(value.tolist())
        else:
            self._update_signal.emit(value)

    def _update_gui(self, value):
        self._paramval_lineedit.setText(str(value))

    def edit_finished(self):
        logging.debug('ArrayEditor edit_finished val={}'.format(
            self._paramval_lineedit.text()))
        params_string = self._paramval_lineedit.text()
        if self.parameter.from_parameter_msg:
            params_string = params_string.replace("'", '"')
        params_list = json.loads(params_string)

        if isinstance(self.parameter.value, array.array):
            if self.parameter.value.typecode == 'q':
                params_list = [int(val) for val in params_list]
            else:
                params_list = [float(val) for val in params_list]
            self.update(array.array(self.parameter.value.typecode, params_list))
        else:
            if Parameter.Type.from_parameter_value(self.parameter.value) \
                    == Parameter.Type.BOOL_ARRAY:
                params_list = [bool(val) for val in params_list]
            elif Parameter.Type.from_parameter_value(self.parameter.value) \
                    == Parameter.Type.BYTE_ARRAY:
                params_list = [bytes(val) for val in params_list]
            else:
                params_list = [str(val) for val in params_list]
            self.update(params_list)

    def _set_to_empty(self):
        self.update('[]')


EDITOR_TYPES = {
    Parameter.Type.BOOL: BooleanEditor,
    Parameter.Type.INTEGER: IntegerEditor,
    Parameter.Type.DOUBLE: DoubleEditor,
    Parameter.Type.STRING: StringEditor,
    Parameter.Type.BOOL_ARRAY: ArrayEditor,
    Parameter.Type.BYTE_ARRAY: ArrayEditor,
    Parameter.Type.INTEGER_ARRAY: ArrayEditor,
    Parameter.Type.DOUBLE_ARRAY: ArrayEditor,
    Parameter.Type.STRING_ARRAY: ArrayEditor,
}
