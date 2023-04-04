#! /usr/bin/env python

class Action(SimpleAction):

    def __init__(self, receiver):

        placeholders = {'<source_frame>': 'footprint',
                        '<target_frame>': 'odom',
                        '<position_x>': 0.0,
                        '<position_y>': 0.0,
                        '<position_z>': 0.0,
                        '<orientation_yaw>': 0.0}

        goal = load_action_from_file(get_package_path('anymal_simple_locomotion_actions') \
                                          + '/actions/climbing/overhanging_obstacle_walk_underneath.yaml', placeholders)

        SimpleAction.__init__(self, receiver, goal)


action = Action(action_loader.execute_steps_relay)
