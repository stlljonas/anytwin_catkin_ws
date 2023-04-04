#! /usr/bin/env python

import os
import rospy
import roslib
from rosparam import load_file, RosParamException
from os.path import *
import free_gait_msgs.msg


class Collection:

    def __init__(self, parameters):
        self.id = None
        self.name = None
        self.action_ids = []
        self.is_sequence = None
        self._initialize(parameters)

    def __str__(self):
        output = 'ID: ' + self.id
        if self.name:
            output = output + ", Name: " + self.name
        if self.action_ids:
            output = output + ", Action IDs: "
            for action_id in self.action_ids:
                output = output + ", " + action_id
        return output

    def _initialize(self, parameters):
        if 'id' in parameters:
            self.id = parameters['id']
        if 'name' in parameters:
            self.name = parameters['name']
        if 'actions' in parameters:
            for action_id in parameters['actions']:
                self.action_ids.append(action_id)
        self.is_sequence = False
        if 'is_sequence' in parameters:
            if parameters['is_sequence']:
                self.is_sequence = True
        if not self.is_sequence:
            self.action_ids = sorted(self.action_ids)

    def to_ros_message(self):
        message = free_gait_msgs.msg.CollectionDescription()
        message.id = self.id
        message.name = self.name
        for action_id in self.action_ids:
            message.action_ids.append(action_id)
        message.is_sequence = self.is_sequence
        return message


class CollectionList:

    def __init__(self, name, free_gait_action_paths):
        self.name = name
        self.free_gait_action_paths = free_gait_action_paths
        self.collections = []
        self.collections_to_merge = []
        self.collections_to_ignore = []

    def update(self):
        self.collections = []
        self.collections_to_merge = []
        self.collections_to_ignore = []
        for path in self.free_gait_action_paths:
            file_path = abspath(join(path, 'collections.yaml'))
            try:
                if not os.path.isfile(file_path):
                    rospy.logdebug('Collections file "' + file_path + '" does not exist, skipping path ...')
                    continue

                rospy.loginfo('Found collections file "' + file_path + '".')
                parameters = load_file(file_path)
                for collections_parameters in parameters[0][0]['collections']:
                    if 'collection' in collections_parameters:
                        collection = Collection(collections_parameters['collection'])
                        self.collections.append(collection)
                    elif 'add_to_collection' in collections_parameters:
                        collection = Collection(collections_parameters['add_to_collection'])
                        self.collections_to_merge.append(collection)
                    elif 'ignore_collection' in collections_parameters:
                        collection = Collection(collections_parameters['ignore_collection'])
                        self.collections_to_ignore.append(collection)

            except Exception:
                rospy.logwarn("Unable to load collections [%s]." % file_path)

        self._merge_collections()
        self._ignore_collections()
        self.collections = sorted(self.collections, key = lambda x: (x.id))
        return True

    def get(self, id):
        collection = [ c for c in self.collections if (c.id == id) ]
        if len(collection) == 0:
            return None
        return collection[0]

    def remove(self, id):
        self.collections[:] = [c for c in self.collections if (c.id != id) ]

    def to_ros_message(self):
        message = []
        for collection in self.collections:
            message.append(collection.to_ros_message())
        return message

    def _merge_collections(self):
        for collection_to_merge in self.collections_to_merge:
            collection = self.get(collection_to_merge.id)
            if not collection:
                rospy.logwarn('Could not find collection with id "%s" to add actions to.'%(collection_to_merge.id))
                continue
            collection.action_ids.extend(collection_to_merge.action_ids)
            collection.action_ids = list(set(collection.action_ids))
            collection.action_ids = sorted(collection.action_ids)

    def _ignore_collections(self):
        for collection_to_ignore in self.collections_to_ignore:
            self.remove(collection_to_ignore.id)
