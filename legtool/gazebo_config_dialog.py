# Copyright 2014 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

SCRIPT_PATH=os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(SCRIPT_PATH, 'build-x86_64'))

import ui_gazebo_config_dialog

class GazeboConfigDialog(QtGui.QDialog):
    def __init__(self, model_name, servo_name_map):
        super(GazeboConfigDialog, self).__init__()
        self.ui = ui_gazebo_config_dialog.Ui_GazeboConfigDialog()
        self.ui.setupUi(self)

        self.ui.modelNameEdit.setText(model_name)

        layout = QtGui.QVBoxLayout()
        layout.setSpacing(0)
        layout.setContentsMargins(0, 0, 0,0)
        self.ui.scrollContents.setLayout(layout)

        self.name_controls = {}

        for key in sorted(servo_name_map.keys()):
            value = servo_name_map[key]

            label = QtGui.QLabel()
            label.setText('ID %d:' % key)

            edit = QtGui.QLineEdit()
            edit.setText(value)

            widget = QtGui.QWidget()
            layout = QtGui.QHBoxLayout(widget)
            layout.addWidget(label)
            layout.addWidget(edit)

            self.ui.scrollContents.layout().addWidget(widget)

            self.name_controls[key] = {
                'widget': widget,
                'label': label,
                'edit': edit,
                }


    def model_name(self):
        return self.ui.modelNameEdit.text()

    def servo_name_map(self):
        result = {}
        for key, value in self.name_controls.iteritems():
            result[key] = value['edit'].text()
        return result
