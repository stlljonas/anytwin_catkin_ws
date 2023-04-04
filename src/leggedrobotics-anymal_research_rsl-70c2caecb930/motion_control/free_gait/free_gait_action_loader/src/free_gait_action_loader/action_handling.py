#! /usr/bin/env python

import os
import rospy
import roslib
from rosparam import load_file, RosParamException
from os.path import *
import free_gait_msgs.msg

class ActionType:
    YAML = 0
    COMBINED_YAML = 1
    PYTHON = 2
    LAUNCH = 3

    @staticmethod
    def to_text(action_type):
        if action_type == ActionType.YAML:
            return 'yaml'
        elif action_type == ActionType.COMBINED_YAML:
            return 'combined_yaml'
        elif action_type == ActionType.PYTHON:
            return 'python'
        elif action_type == ActionType.LAUNCH:
            return 'launch'
        else:
            return None

    @staticmethod
    def from_text(action_type):
        if action_type == 'yaml':
            return ActionType.YAML
        elif action_type == 'combined_yaml':
            return ActionType.COMBINED_YAML
        elif action_type == 'python':
            return ActionType.PYTHON
        elif action_type == 'launch':
            return ActionType.LAUNCH
        else:
            return None


class ActionEntry:

    def __init__(self, path, parameters):
        self.id = None
        self.name = None
        self.file = None
        self.type = None
        self.description = None
        self.directory = None
        self._initialize(path, parameters);

    def __str__(self):
        output = 'ID: ' + self.id
        if self.name:
            output = output + ", Name: " + self.name
        if self.file:
            output = output + ", File: " + self.file
        if self.file:
            output = output + ", Type: " + ActionType.to_text(self.type)
        return output

    def _initialize(self, path, parameters):
        if 'id' in parameters:
            self.id = parameters['id']
        if 'name' in parameters:
            self.name = parameters['name']
        if 'file' in parameters:
            self.file = abspath(join(path, parameters['file']))
        if 'type' in parameters:
            self.type = ActionType.from_text(parameters['type'])
        if 'description' in parameters:
            self.description = parameters['description']
        self.directory = dirname(abspath(self.file))

    def to_ros_message(self):
        message = free_gait_msgs.msg.ActionDescription()
        message.id = self.id
        message.name = self.name
        message.file = self.file
        message.type =  ActionType.to_text(self.type)
        message.description = self.description
        return message


class ActionList:

    def __init__(self, name, free_gait_action_paths):
        self.name = name
        self.actions = []
        self.free_gait_action_paths = free_gait_action_paths

    def update(self):
        self.actions = []
        for path in self.free_gait_action_paths:
            file_path = abspath(join(path, 'actions.yaml'))
            try:
                if not os.path.isfile(file_path):
                    rospy.logdebug('Actions file "' + file_path + '" does not exist, skipping path ...')
                    continue

                rospy.loginfo('Found actions file "' + file_path + '".')
                parameters = load_file(file_path)
                for action_parameters in parameters[0][0]['actions']:
                    duplicate, index = self.duplicateId(action_parameters['action']['id'])
                    if duplicate:
                        rospy.loginfo('Action with ID "' + action_parameters['action']['id'] + '" is overlaid.')
                        self.actions[index] = ActionEntry(path, action_parameters['action'])
                    else:
                        entry = ActionEntry(path, action_parameters['action'])
                        self.actions.append(entry)

            except Exception:
                rospy.logwarn("Unable to load actions [%s]." % file_path)

        self.actions = sorted(self.actions, key = lambda x: (x.id))
        return True

    def duplicateId(self, id):
        for i in range(len(self.actions)):
            if self.actions[i].id == id:
                return True, i
        return False, 0

    def get(self, id):
        entry = [ e for e in self.actions if (e.id == id) ]
        if len(entry) == 0:
            return None
        return entry[0]

    def get_multiple(self, ids):
        entries = []
        for id in ids:
            entry = self.get(id)
            if entry:
                entries.append(entry)
        return entries

    def to_ros_message(self, ids = []):
        actions = []
        if len(ids):
            actions = self.get_multiple(ids)
        else:
            actions = self.actions
        message = []
        for action in actions:
            message.append(action.to_ros_message())
        return message
