#! /usr/bin/env python

class Action(SimpleAction):

    def __init__(self, receiver):

        placeholders = {'<source_frame>': 'footprint',
                        '<target_frame>': 'odom',
                        '<position_x>': 0.0,
                        '<position_y>': 0.0,
                        '<position_z>': 0.0,
                        '<orientation_yaw>': 0.0,
                        '<step_height>': -0.20}

        goal = load_action_from_file(get_package_path('anymal_simple_locomotion_actions') \
                                          + '/actions/climbing/step_climb_down_reverse.yaml', placeholders)

        SimpleAction.__init__(self, receiver, goal)


action = Action(action_loader.execute_steps_relay)
