## *********************************************************
## 
## File autogenerated for the htwAalen_signDetect package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

##**********************************************************
## Software License Agreement (BSD License)
##
##  Copyright (c) 2008, Willow Garage, Inc.
##  All rights reserved.
##
##  Redistribution and use in source and binary forms, with or without
##  modification, are permitted provided that the following conditions
##  are met:
##
##   * Redistributions of source code must retain the above copyright
##     notice, this list of conditions and the following disclaimer.
##   * Redistributions in binary form must reproduce the above
##     copyright notice, this list of conditions and the following
##     disclaimer in the documentation and/or other materials provided
##     with the distribution.
##   * Neither the name of the Willow Garage nor the names of its
##     contributors may be used to endorse or promote products derived
##     from this software without specific prior written permission.
##
##  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
##  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
##  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
##  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
##  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
##  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
##  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
##  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
##  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
##  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
##  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
##  POSSIBILITY OF SUCH DAMAGE.
##**********************************************************/

config_description = [{'srcline': 9, 'description': 'A double parameter', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'double_param0', 'edit_method': '', 'default': 0.01, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 10, 'description': 'A double parameter', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'double_param1', 'edit_method': '', 'default': 0.01, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 11, 'description': 'A double parameter', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'double_param2', 'edit_method': '', 'default': 0.01, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 12, 'description': 'A double parameter', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'double_param3', 'edit_method': '', 'default': 0.01, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 13, 'description': 'A double parameter', 'max': 100.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'double_param4', 'edit_method': '', 'default': 0.01, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 14, 'description': 'A double parameter', 'max': 100.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'double_param5', 'edit_method': '', 'default': 0.01, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 15, 'description': 'A double parameter', 'max': 10000.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'double_param6', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 16, 'description': 'A double parameter', 'max': 100.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'double_param7', 'edit_method': '', 'default': 0.01, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 19, 'description': 'Use a custom distance for advisor', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'use_custom_distance', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 20, 'description': 'Enter a custom distance', 'max': 10000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'custom_distance', 'edit_method': '', 'default': 5000, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 32, 'description': 'Select Distance', 'max': 10000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'distance', 'edit_method': "{'enum_description': 'Distance Selector', 'enum': [{'srcline': 22, 'description': '0.5 meter', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'cconsttype': 'const int', 'value': 500, 'ctype': 'int', 'type': 'int', 'name': '0_5meter'}, {'srcline': 23, 'description': '1 meter', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'cconsttype': 'const int', 'value': 1000, 'ctype': 'int', 'type': 'int', 'name': '1meter '}, {'srcline': 24, 'description': '2 meters', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'cconsttype': 'const int', 'value': 2000, 'ctype': 'int', 'type': 'int', 'name': '2meter'}, {'srcline': 25, 'description': '3 meters', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'cconsttype': 'const int', 'value': 3000, 'ctype': 'int', 'type': 'int', 'name': '3meter'}, {'srcline': 26, 'description': '4 meters', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'cconsttype': 'const int', 'value': 4000, 'ctype': 'int', 'type': 'int', 'name': '4meter'}, {'srcline': 27, 'description': '5 meters', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'cconsttype': 'const int', 'value': 5000, 'ctype': 'int', 'type': 'int', 'name': '5meter'}, {'srcline': 28, 'description': '6 meters', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'cconsttype': 'const int', 'value': 6000, 'ctype': 'int', 'type': 'int', 'name': '6meter'}, {'srcline': 29, 'description': '7 meters', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'cconsttype': 'const int', 'value': 7000, 'ctype': 'int', 'type': 'int', 'name': '7meter'}, {'srcline': 30, 'description': '8 meters', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'cconsttype': 'const int', 'value': 8000, 'ctype': 'int', 'type': 'int', 'name': '8meter'}]}", 'default': 500, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 39, 'description': 'Select PCLPicture', 'max': 1, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'pcl_picture', 'edit_method': "{'enum_description': 'PCLPicture', 'enum': [{'srcline': 35, 'description': 'Advisor', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'Advisor'}, {'srcline': 36, 'description': 'RGB', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'RGB'}]}", 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 48, 'description': 'Select PCL Picture Grid', 'max': 3, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'pcl_picture_grid', 'edit_method': "{'enum_description': 'PCLPictureGrid', 'enum': [{'srcline': 42, 'description': 'Off', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'Off'}, {'srcline': 43, 'description': 'Columns', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'Columns'}, {'srcline': 44, 'description': 'Lines', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'Lines'}, {'srcline': 45, 'description': 'Both', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': 'Both'}]}", 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 49, 'description': 'Enter grid spacing', 'max': 100, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'pcl_picture_grid_space', 'edit_method': '', 'default': 50, 'level': 0, 'min': 2, 'type': 'int'}, {'srcline': 53, 'description': 'Enable single row highlight', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'pcl_highlight_row_enable', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 54, 'description': 'Select specific row', 'max': 480, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'pcl_highlight_row', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 57, 'description': 'Enable single column highlight', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'pcl_highlight_col_enable', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 58, 'description': 'Select specific column', 'max': 640, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'pcl_highlight_col', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 61, 'description': 'Enable setting one row to zero', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'pcl_zero_row_enable', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 62, 'description': 'Select specific row', 'max': 480, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'pcl_zero_row', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 65, 'description': 'Enable setting one column to zero', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'pcl_zero_col_enable', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 66, 'description': 'Select specific column', 'max': 640, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'pcl_zero_col', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 70, 'description': 'Enable Filtertest', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'pcl_filter_test', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 73, 'description': 'Stop output of new point clouds', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'pcl_stop_output', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 76, 'description': 'Print values', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'pcl_gather_values', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 79, 'description': 'Print values', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'pcl_value_print', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 82, 'description': 'Capture step map subtracted from distance value', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'pcl_value_catch', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 85, 'description': 'Subtract saved disturbance map', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'pcl_subtr_disturb', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 90, 'description': 'Fetch values for statistics', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'fetchValues', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 91, 'description': 'Output values for statistics', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '../cfg/disturbance_filter_calibrator.cfg', 'name': 'outputValues', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}]

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

for param in config_description:
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

disturbance_filter_calibrator_0_5meter = 500
disturbance_filter_calibrator_1meter  = 1000
disturbance_filter_calibrator_2meter = 2000
disturbance_filter_calibrator_3meter = 3000
disturbance_filter_calibrator_4meter = 4000
disturbance_filter_calibrator_5meter = 5000
disturbance_filter_calibrator_6meter = 6000
disturbance_filter_calibrator_7meter = 7000
disturbance_filter_calibrator_8meter = 8000
disturbance_filter_calibrator_Advisor = 0
disturbance_filter_calibrator_RGB = 1
disturbance_filter_calibrator_Off = 0
disturbance_filter_calibrator_Columns = 1
disturbance_filter_calibrator_Lines = 2
disturbance_filter_calibrator_Both = 3
