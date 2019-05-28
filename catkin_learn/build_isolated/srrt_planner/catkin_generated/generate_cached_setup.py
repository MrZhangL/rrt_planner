# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/kinetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/kinetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in "/home/zhang/catkin_learn/devel_isolated/rrt_exploration_tutorials;/home/zhang/catkin_learn/devel_isolated/rrt_exploration;/home/zhang/catkin_learn/devel_isolated/robot_move;/home/zhang/catkin_learn/devel_isolated/path_planning;/home/zhang/catkin_learn/devel_isolated/navigation;/home/zhang/catkin_learn/devel_isolated/mbot_bringup;/home/zhang/catkin_learn/devel_isolated/mbot;/home/zhang/catkin_learn/devel_isolated/learning_tf;/home/zhang/catkin_learn/devel_isolated/learning_communication;/home/zhang/catkin_wos/devel;/home/zhang/catkin_learn/devel;/home/zhang/catkin_ws/devel;/opt/ros/kinetic".split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/zhang/catkin_learn/devel_isolated/srrt_planner/env.sh')

output_filename = '/home/zhang/catkin_learn/build_isolated/srrt_planner/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    #print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
