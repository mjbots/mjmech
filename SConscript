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

Import('canonenv')
env = canonenv.Clone()

env.Command('ui_legtool_main_window.py', 'legtool_main_window.ui',
            'pyside-uic $SOURCE -o $TARGET')

env.Command('legtool/tabs/ui_gazebo_config_dialog.py',
            'legtool/tabs/gazebo_config_dialog.ui',
            'pyside-uic $SOURCE -o $TARGET')
