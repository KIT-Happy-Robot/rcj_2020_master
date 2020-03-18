#!/usr/bin/env python
# -*- coding: utf-8 -*-
#----------------------------------------------------------
# Title: RCJ2020_GoGetItの競技プログラム
# Author: Issei Iida
# Date: 2020/02/24
#----------------------------------------------------------

# Python
import sys
# ROS
import rospy
import rosparam
import smach
import smach_ros
# Message
from std_msgs.msg import String
from std_srvs.srv import Empty
from voice_common_pkg.srv import GgiLearning 

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_action_client import navigationAC, enterTheRoomAC, exeActionPlanAC
from common_function import speak, m6Control, searchLocationName


class Training(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['training_finish'])
        # Service
        self.training_srv = rospy.ServiceProxy('/ggi_training_phase', Empty)

    def execute(self, userdata):
        rospy.loginfo('Executing state: ENTER')
        m6Control(0.0)
        speak('Start GoGetIt')
        # enterTheRoomAC(0.8)
        self.training_srv()
        speak('Start TestPhase')
        return 'training_finish'


class DecideMove(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['decide_finish'])
        # Subscriber
        self.posi_sub = rospy.Subscriber('/navigation/move_place', String, self.currentPosiCB)
        # Value
        # self.operator_coord = searchLocationName('operator')
        self.current_position = 'operator'

    def currentPosiCB(self, data):
        self.current_position = data.data

    def execute(self, userdata):
        rospy.loginfo('Executing state: DECIDE_MOVE')
        if self.current_position != 'operator':
            operator_coord = searchLocationName('operator')
            navigationAC(operator_coord)
            speak('I arrived operator')
        else:
            pass
        return 'decide_finish'


class ListenCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['listen_success',
                                         'listen_failure',
                                         'next_cmd',
                                         'all_cmd_finish'],
                             output_keys = ['cmd_out'])
        # ServiceProxy
        # self.listen_srv = rospy.ServiceProxy('/gpsr/actionplan', ActionPlan)
        self.ggi_listen_srv = rospy.ServiceProxy('/test_phase', GgiLearning)
        # Value
        self.listen_count = 1
        self.cmd_count = 1

    def execute(self, userdata):
        rospy.loginfo('Executing state: LISTEN_COMMAND')
        if self.cmd_count == 4:
            speak('Finish all command')
            speak('Finish GoGetIt')
            return 'all_cmd_finish'
        elif self.listen_count <= 3:
            speak('CommandNumber is ' + str(self.cmd_count))
            speak('ListenCount is ' + str(self.listen_count))
            speak('Please instruct me')
            # result = self.listen_srv()
            location = self.ggi_listen_srv().location_name
            result = True
            if result:
            # if result.result:
                self.listen_count = 1
                self.cmd_count += 1
                # userdata.cmd_out = result
                userdata.cmd_out = location
                return 'listen_success'
            else:
                self.listen_count += 1
                speak("I could't listen")
                return 'listen_failure'
        else:
            speak("I couldn't understand the instruction")
            self.listen_count = 1
            self.cmd_count +=1
            return 'next_cmd'


class ExeAction(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['action_success',
                                         'action_failure'],
                             input_keys = ['cmd_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: EXE_ACTION')
        # action = userdata.cmd_in.action
        # data = userdata.cmd_in.data
        name = userdata.cmd_in
        action = ['go','grasp','go','give']
        data = [name,'any','operator','any']
        result = exeActionPlanAC(action, data)
        if result:
            return 'action_success'
        else:
            return 'action_failure'


def main():
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])
    with sm_top:
        smach.StateMachine.add(
                'TRAINING',
                Training(),
                transitions = {'training_finish':'DECIDE_MOVE'})

        smach.StateMachine.add(
                'DECIDE_MOVE',
                DecideMove(),
                transitions = {'decide_finish':'LISTEN_COMMAND'})

        smach.StateMachine.add(
                'LISTEN_COMMAND',
                ListenCommand(),
                transitions = {'listen_success':'EXE_ACTION',
                               'listen_failure':'LISTEN_COMMAND',
                               'next_cmd':'DECIDE_MOVE',
                               'all_cmd_finish':'finish_sm_top'},
                remapping = {'cmd_out':'cmd'})

        smach.StateMachine.add(
                'EXE_ACTION',
                ExeAction(),
                transitions = {'action_success':'DECIDE_MOVE',
                               'action_failure':'DECIDE_MOVE'},
                remapping = {'cmd_in':'cmd'})

    outcomes = sm_top.execute()


if __name__ == '__main__':
    rospy.init_node('sm_ggi', anonymous = True)
    main()
