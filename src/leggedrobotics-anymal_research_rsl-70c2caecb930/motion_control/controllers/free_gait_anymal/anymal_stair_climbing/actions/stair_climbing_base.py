#! /usr/bin/env python

import math
import free_gait

'''
                                                   Nr 4 (last step nr)
                                                +---------------------+
                                                |
     Front legs determine                       |
     the current step number.            Nr 3   |
                                      +---------+
                     Run              |
                  <-------->.         |
                  .         .  Nr 2   |
                  .         +---------+
                  .         |
                  .         |
                  .  Nr 1   |
                  +---------+....
                  |             ^
                  ^             | Rise
            Nr 0  |z            |
 -----------------+-->.x........v
              Stairs frame
'''


class Action(free_gait.CombinedYamlAction):

    def __init__(self, relay):
        free_gait.CombinedYamlAction.__init__(self, relay)
        self.yaml_object = free_gait.CombinedYamlActionDefinition()
        self.robot_parameters = {}
        self.stairs_parameters = {}

        self.generate_default_stairs(2)
        self.generate_motion()

    def set_default_robot_parameters(self):
        self.robot_parameters['base_height'] = 0.47
        self.robot_parameters['foot_placement_width'] = 0.48
        self.robot_parameters['average_foot_velocity'] = 0.55
        self.robot_parameters['target_frame'] = 'map'

    def set_default_stairs_parameters(self):
        self.stairs_parameters['package'] = 'anymal_stair_climbing'
        self.stairs_parameters['folder'] = 'actions/climb_up_forward'
        self.stairs_parameters['source_frame'] = 'stairs'
        self.stairs_parameters['position_x'] = 0.0
        self.stairs_parameters['position_y'] = 0.0
        self.stairs_parameters['position_z'] = 0.0
        self.stairs_parameters['orientation_yaw'] = 0.0
        self.stairs_parameters['rise'] = 0.17
        self.stairs_parameters['run'] = 0.29

    def generate_first_step(self, rise=None, run=None, package=None, folder=None):
        if rise is None:
            rise = self.stairs_parameters['rise']
        if run is None:
            run = self.stairs_parameters['run']
        if package is None:
            package = self.stairs_parameters['package']
        if folder is None:
            folder = self.stairs_parameters['folder']

        self.yaml_object.append_action(package, folder + '/0_start.yaml')

    def generate_default_stairs(self, number_of_steps):
        self.set_default_robot_parameters()
        self.set_default_stairs_parameters()
        self.generate_first_step()

    def generate_motion(self):
        self.set_goal_from_yaml(self.yaml_object.yaml_object)


action = Action(action_loader.execute_steps_relay)
