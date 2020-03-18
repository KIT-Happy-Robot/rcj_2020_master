#!/usr/bin/env python
# -*- coding: utf-8 -*-
#------------------------------------------------------
#Title:
#Author:Ishiyama YUki
#data: 2020/3/18
#Memo
#------------------------------------------------------
import sys

import rospy

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_action_client import *
from common_function import *

def main(self):
    speak('start robot inspection')
    enterTheRoomAC(0.8)
    location_list = serchLocationName('goal')
    navigationAC(location_list)
    return 0

if __name__ == '__main__':
    rospy.init_node('RobotInspection')
    main()
    

