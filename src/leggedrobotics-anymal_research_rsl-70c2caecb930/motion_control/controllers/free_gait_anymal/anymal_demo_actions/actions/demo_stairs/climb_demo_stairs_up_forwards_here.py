#! /usr/bin/env python

import math

class Action(SimpleAction):

    def __init__(self, receiver):

        placeholders = {'<source_frame>': 'footprint',
                        '<target_frame>': 'odom',
                        '<position_x>': 0.5, # The yaml file assumes that the stairs start at x=0.0.
                        '<position_y>': 0.0,
                        '<position_z>': 0.0,
                        '<orientation_yaw>': 0.0,
                        '<base_height>': 0.50,
                        '<support_margin_small>': 0.04,
                        '<support_margin_medium>': 0.04,
                        '<support_margin_large>': 0.06,
                        '<average_foot_velocity>': 0.55,
                        '<average_foot_velocity_slow>': 0.45,
                        '<average_base_linear_velocity>': 0.1,
                        '<average_base_angular_velocity>': 0.2}

        goal = load_action_from_file(get_package_path('anymal_demo_actions') \
                                     + '/actions/demo_stairs/climb_demo_stairs_up_forwards.yaml', placeholders)

        SimpleAction.__init__(self, receiver, goal)


action = Action(action_loader.execute_steps_relay)
