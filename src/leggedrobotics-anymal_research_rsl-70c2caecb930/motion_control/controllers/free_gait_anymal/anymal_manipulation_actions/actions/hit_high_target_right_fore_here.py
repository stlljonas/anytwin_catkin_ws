#! /usr/bin/env python

import math

class Action(SimpleAction):

    def __init__(self, receiver):

        placeholders = {'<source_frame>': 'footprint',
                        '<position_x>': 0.6,
                        '<position_y>': -0.2,
                        '<position_z>': 0.8,
                        '<wait_time_at_position>': 0.0}

        goal = load_action_from_file(get_package_path('anymal_manipulation_actions') \
                                          + '/actions/hit_high_target_right_fore_template.yaml', placeholders)

        SimpleAction.__init__(self, receiver, goal)


action = Action(action_loader.execute_steps_relay)
