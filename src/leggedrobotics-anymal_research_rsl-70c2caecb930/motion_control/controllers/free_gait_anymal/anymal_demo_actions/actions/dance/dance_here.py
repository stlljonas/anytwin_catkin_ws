#! /usr/bin/env python

import math
import os

class Action(CombinedYamlAction):

    def __init__(self, receiver):
        self.state = ActionState.UNINITIALIZED
        self.feedback_callback = None
        self.done_callback = None
        self.relay = receiver
        self.goal = None
        self.feedback = None
        self.timeout = rospy.Duration()

        BPM = 84.0
        BPM/60.0
        DeltaT = 60.0/BPM*2.0

        placeholders = {'<source_frame>': 'footprint',
                        '<target_frame>': 'odom',
                        '<position_x>': 0.0,
                        '<position_y>': 0.0,
                        '<position_z>': 0.0,
                        '<orientation_yaw>': 0.0,
                        '<average_foot_velocity>': 1.0,
                        '<DeltaT>': DeltaT,
                        '<DeltaT18>': DeltaT/8.0,
                        '<DeltaT14>': DeltaT/4.0,
                        '<DeltaT24>': DeltaT/2.0,
                        '<DeltaT34>': DeltaT*3.0/4.0,
                        '<DeltaT54>': DeltaT*5.0/4.0,}

        file_path = get_package_path('anymal_demo_actions') + '/actions/dance/dance.yaml'

        from rosparam import load_file
        if not os.path.isfile(file_path):
            rospy.logerr('File with path "' + file_path + '" does not exists.')
            self.set_state(ActionState.ERROR)
            return
        yaml_object = load_file(file_path)
        yaml_object[0][0]["global_placeholders"] = placeholders;
        print(yaml_object[0][0])

        self.set_goal_from_yaml(yaml_object)


action = Action(action_loader.execute_steps_relay)
