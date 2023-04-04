#! /usr/bin/env python

import anymal_msgs.msg
from rosparam import load_file

class Action(ActionBase):

    def __init__(self, relay):
        ActionBase.__init__(self, relay)

    def start(self):
        try:
            anymal_state = rospy.wait_for_message("/state_estimator/anymal_state",
                                                     anymal_msgs.msg.AnymalState, 1.0)
            feet_positions = self._get_feet_positions_from_state(anymal_state, 'map')
            [base_position, base_orientation] = self._get_base_pose_from_state(anymal_state, 'map')
            self._create_action(feet_positions, base_position, base_orientation)
            self.set_state(ActionState.SUCCEEDED)
        except rospy.exceptions.ROSException:
            rospy.logerr("[return_to_pose_save] Could not determine robot's pose.")
            self.set_state(ActionState.ERROR)


    def _get_feet_positions_from_state(self, anymal_state, desired_frame_id):
        feet_positions_odom = [[position.x, position.y, position.z] for position in \
                               [contact.position for contact in anymal_state.contacts]]
        feet_positions_in_desired_frame = [transform_coordinates(anymal_state.header.frame_id, \
                              desired_frame_id, position)[0] for position in feet_positions_odom]
        return feet_positions_in_desired_frame


    def _get_base_pose_from_state(self, anymal_state, desired_frame_id):
        position_in_odom = [anymal_state.pose.pose.position.x, \
                            anymal_state.pose.pose.position.y, \
                            anymal_state.pose.pose.position.z]
        orientation_in_odom = [anymal_state.pose.pose.orientation.x, \
                               anymal_state.pose.pose.orientation.y, \
                               anymal_state.pose.pose.orientation.z, \
                               anymal_state.pose.pose.orientation.w]
        return transform_coordinates(anymal_state.header.frame_id, \
                              desired_frame_id, position_in_odom, orientation_in_odom)


    def _create_action(self, feet_positions, base_position, base_orientation):
        placeholders = {'<lf_x>': feet_positions[0][0],
                        '<lf_y>': feet_positions[0][1],
                        '<lf_z>': feet_positions[0][2],
                        '<rf_x>': feet_positions[1][0],
                        '<rf_y>': feet_positions[1][1],
                        '<rf_z>': feet_positions[1][2],
                        '<lh_x>': feet_positions[2][0],
                        '<lh_y>': feet_positions[2][1],
                        '<lh_z>': feet_positions[2][2],
                        '<rh_x>': feet_positions[3][0],
                        '<rh_y>': feet_positions[3][1],
                        '<rh_z>': feet_positions[3][2],
                        '<base_pos_x>': base_position[0],
                        '<base_pos_y>': base_position[1],
                        '<base_pos_z>': base_position[2],
                        '<base_rot_x>': base_orientation[0],
                        '<base_rot_y>': base_orientation[1],
                        '<base_rot_z>': base_orientation[2],
                        '<base_rot_w>': base_orientation[3]}

        template_file_path = get_package_path('anymal_simple_locomotion_actions') \
                             + '/actions/return_to_pose/return_to_pose_template.yaml'
        new_file_path = get_package_path('anymal_simple_locomotion_actions') \
                                     + '/actions/return_to_pose/return_to_pose.yaml'

        with open(template_file_path) as infile, open(new_file_path, 'w') as outfile:
            for line in infile:
                for source, target in placeholders.iteritems():
                    line = line.replace(source, str(target))
                outfile.write(line)


action = Action(action_loader.execute_steps_relay)
