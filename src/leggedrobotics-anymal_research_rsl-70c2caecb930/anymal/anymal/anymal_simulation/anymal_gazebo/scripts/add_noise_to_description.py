#!/usr/bin/python
#
# @authors     Jonathan Klein Schiphorst, Stephane Caron
# @affiliation ANYbotics
# @brief       Add noise to URDF parameters and push updated model to new rosparam used to spawn robot in Gazebo

import numpy as np
import roslib

roslib.load_manifest('urdfdom_py')

import rospy
from urdf_parser_py.urdf import URDF

NODE_NAME = 'add_noise_to_description'


def get_node_param(param_name):
    return rospy.get_param('{}/{}'.format(NODE_NAME, param_name))


def has_node_param(param_name):
    return rospy.has_param('{}/{}'.format(NODE_NAME, param_name))


def set_node_param(param_name, value):
    return rospy.set_param('{}/{}'.format(NODE_NAME, param_name), value)


def multi_hasattr(obj, attr):
    """
    Check attributes are present in an object.

    :returns: ``multi_hasattr(x, 'a.b.c.d') returns True if object x has nested attribute x.a.b.c.d
    """
    attributes = attr.split(".")
    for i in attributes:
        try:
            obj = getattr(obj, i)
        except AttributeError:
            pass
    if obj:
        return True
    else:
        return False


def multi_getattr(obj, attr):
    """
    Get nested attributes in an object.

    :returns: multi_getattr(x, 'a.b.c.d') is equivalent to x.a.b.c.d, but can search in nested objects.
    """
    attributes = attr.split(".")
    for i in attributes:
        try:
            obj = getattr(obj, i)
        except AttributeError:
            pass
    return obj


def multi_setattr(obj, attr, setValue):
    """
    Sets a nested attribute ``obj.a.b.c.d`` to ``setValue``.

    :param obj: Object.
    :param attr: Attribute to set.
    :param setValue: New value for attribute.
    """
    attributes = attr.split(".")
    for i in range(len(attributes)-1):
        try:
            obj = getattr(obj, attributes[i])
        except AttributeError:
            pass
    setattr(obj, attributes[-1], setValue)


class UrdfManipulator:
    """
    Update URDF model:
    - Read out URDF from parameter server.
    - Add noise specified in config/description_noise.yaml to parameters.
    - Write URDF with added noise to parameter server.
    A seed can be set, or will be generated if seed == 0, to generate same noise over multiple runs.
    """

    def __init__(self, verbose=True):
        """
        Parse URDF into python structure and calls setSeed method.

        :param verbose: Verbosity level.
        """
        self.set_seed()
        self.urdf = URDF.from_parameter_server(get_node_param('description_name'))  # parses urdf to python object.
        self.verbose = verbose

    def set_seed(self):
        """
        Set the seed for generating the Gaussian noise.

        If no seed is given (seed == 0), one will be generated and stored in the parameter server.
        """
        if has_node_param('seed'):
            seed = int(get_node_param('seed'))
        else:  # no seed
            seed = np.random.randint(1, 2**31) # Pick integer from 1 to 2^31. Max range of np.random.seed is 2^31.
            set_node_param('seed', seed)
        np.random.seed(seed)
        self.seed = seed

    def add_gaussian_noise(self, noise_factor, attributes, name=None, offset=None):
        """
        Add Gaussian noise to a link parameter. The noise is proportional to the original parameter.

        :param noise_factor: Scale of noise. Link nominal parameter is roughly multiplited by this factor.
        :param attributes: Attributes to update.
        :param name: If provided, that particular links parameter gets altered, otherwise all links that possess the parameter get altered.
        :param offset: Optional offset to apply to the same parameter.
        """
        if name is not None:
            links = (selected_links for selected_links in self.urdf.links if selected_links.name == name)
        else:  # name is None
            links = (selected_links for selected_links in self.urdf.links if multi_hasattr(selected_links, attributes))
        for link in links:
            parameter = multi_getattr(link, attributes)
            sigma = noise_factor * parameter if np.size(parameter) == 1 else [element * noise_factor for element in parameter]
            noise = sigma * np.random.normal(0,1) if np.size(parameter) == 1 else sigma * np.random.normal(0, 1, np.size(parameter))
            if self.verbose:
                print("Adding to '{}' of link '{}':".format(attributes, link.name) +
                      (" offset={}".format(offset) if offset is not None else "") +
                      " noise={}".format(noise))
            new_parameter = parameter + noise  # careful here: ``parameter`` is a list but ``noise`` is an array
            if offset is not None:
                if np.size(offset) != np.size(parameter):
                    raise ValueError("offset for '{}' of '{}' has incorrect number of elements".format(attributes, link.name))
                new_parameter += offset
            multi_setattr(link, attributes, new_parameter)

    def update_joint(self, joint_name, param_name, offset):
        """
        Add Gaussian noise to a link parameter. The noise is proportional to the original parameter.

        :param joint_name: Joint name in URDF model.
        :param param_name: Name of parameter to update.
        :param value: Offset to add to the parameter value.
        """
        joints = [joint for joint in self.urdf.joints if joint.name == joint_name]
        if len(joints) != 1:
            print("Joint '{}' was not found, skipping it. Check your configuration.".format(joint_name))
        joint = joints[0]
        old_value = np.array(multi_getattr(joint, param_name))  # also checks that param exists
        new_value = old_value + offset
        if self.verbose:
            print("Updating '{}' of joint '{}' ".format(param_name, joint.name) +
                  " from {} to {}".format(old_value, new_value))
        multi_setattr(joint, param_name, new_value)

    def set_new_urdf(self):
        """Publish new URDF to parameter server."""
        new_xml = self.urdf.to_xml_string()
        simulation_description = get_node_param('simulation_description_name')
        rospy.set_param('/{}'.format(simulation_description), new_xml)
        print("[add_noise_to_description] Set updated URDF to '{}'".format(simulation_description))


def main():
    """Main function."""
    rospy.init_node(NODE_NAME)
    if rospy.get_param(NODE_NAME):
        robot = UrdfManipulator()
        if has_node_param('all_links'):  # adjust certain parameter for all links
            all_links = get_node_param('all_links')
            for all_link in all_links:
                robot.add_gaussian_noise(all_link['noise_factor'], all_link['parameter'])
        if has_node_param('specific_links'):  # adjust certain parameters for specific link
            specific_links = get_node_param('specific_links')
            for link in specific_links:
                noise_factor = link['noise_factor'] if 'noise_factor' in link else 0.
                offset = link['offset'] if 'offset' in link else None
                robot.add_gaussian_noise(noise_factor, link['parameter'], name=link['name'], offset=offset)
        if has_node_param('specific_joints'):
            specific_joints = get_node_param('specific_joints')
            for joint in specific_joints:
                robot.update_joint(joint['name'], joint['parameter'], joint['offset'])
        robot.set_new_urdf()
    else:  # don't adjust parameters and use same URDF for control and simulation.
        simulation_description = get_node_param('simulation_description_name')
        original_description = get_node_param('description_name')
        print("[add_noise_description] '{}' URDF forwarded as is to '{}'".format(
            original_description, simulation_description))
        rospy.set_param(simulation_description, get_node_param(original_description)) # Load unaltered URDF to parameter for simulation


if __name__ == '__main__':
    main()
