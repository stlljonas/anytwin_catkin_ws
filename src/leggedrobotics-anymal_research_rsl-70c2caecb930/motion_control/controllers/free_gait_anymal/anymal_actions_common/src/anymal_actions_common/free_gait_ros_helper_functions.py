#! /usr/bin/env python

#################################################################################
# help_functions.py
#
# Helper functions for free gait python actions.
#
# Creation Date: 2020-05-05
# Author: Valentin Yuryev, vyuryev@anybotics.com
# ANYbotics AG
#################################################################################

import any_msgs.srv
from any_state_estimator_msgs.srv import ResetStateEstimator
import anymal_msgs.msg
from control_unit_msgs.msg import ActiveController
import rospy
import std_srvs.srv


def all_feet_in_open_contact(timeout=0.5):
    # Check whether all feet have open contact states.
    try:
        feet_contacts = get_feet_contacts(timeout)
        for contact in feet_contacts:
            if contact != anymal_msgs.msg.Contact.STATE_OPEN:
                return False
    except rospy.exceptions.ROSException:
        rospy.logerr("ANYmal state not available.")
        return False
    return True


def all_feet_in_contact(timeout=0.5):
    # Check whether all feet have closed contact states
    try:
        feet_contacts = get_feet_contacts(timeout)
        for contact in feet_contacts:
            if contact != anymal_msgs.msg.Contact.STATE_CLOSED:
                return False
    except rospy.exceptions.ROSException:
        rospy.logerr("ANYmal state not available.")
        return False
    return True


def get_feet_contacts(timeout=0.5):
    # Return the contacts
    try:
        anymal_state = get_anymal_state(timeout)
        feet_contacts = [contact.state for contact in anymal_state.contacts]
    except rospy.exceptions.ROSException:
        rospy.logerr("ANYmal state not available.")
        return None
    return feet_contacts

def get_feet_positions(timeout=0.5):
    # Return the contacts
    try:
        anymal_state = get_anymal_state(timeout)
        feet_positions = [[position.x, position.y, position.z] for position in
                               [contact.position for contact in anymal_state.contacts]]
    except rospy.exceptions.ROSException:
        rospy.logerr("ANYmal state not available.")
        return None
    return feet_positions


def reset_free_gait_controller(timeout=0.1):
    service_name = '/anymal_ctrl_free_gait/reset'
    rospy.wait_for_service(service_name, timeout)
    try:
        reset_locomotion_controller = rospy.ServiceProxy(service_name, std_srvs.srv.Trigger)
        response = reset_locomotion_controller()
        return response.success
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)
        return False


def reset_state_estimator(service_name, service, timeout=0.1):
    try:
        rospy.wait_for_service(service_name, timeout)
    except rospy.ROSException, e:
        rospy.loginfo("Could not find" + service_name + "topic, assuming simulated estimator.")
        return True
    try:
        reset_state_estimator_srv = rospy.ServiceProxy(service_name, service)
        reset_state_estimator_srv()
        return True
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)
        return False


def reset_state_estimator_here(timeout=0.1):
    return reset_state_estimator('/state_estimator/reset_here', std_srvs.srv.Trigger, timeout)


def reset_state_estimator_origin(timeout=0.1):
    return reset_state_estimator('/state_estimator/reset', ResetStateEstimator, timeout)


def set_checking_for_state_estimator(enable, timeout=0.1):
    service_name = "/anymal_ctrl_free_gait/check_state_estimator"
    rospy.wait_for_service(service_name, timeout)
    try:
        check_state_estimator = rospy.ServiceProxy(service_name, any_msgs.srv.Toggle)
        response = check_state_estimator(enable)
        return response.success
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)
        return False


def set_zero_velocity_update_for_state_estimator(enable, timeout=0.1):
    service_name = '/state_estimator/toggle_zero_velocity_updates'
    try:
        rospy.wait_for_service(service_name, timeout)
        try:
            zero_velocity_updates_for_state_estimator = rospy.ServiceProxy(service_name, any_msgs.srv.Toggle)
            response = zero_velocity_updates_for_state_estimator(enable)
            return response.success
        except rospy.ServiceException, e:
            rospy.logerror("Service call failed: %s" % e)
            return False
    except rospy.ROSException, e:
        rospy.loginfo("Did not activate zero velocity updates for state estimator, assuming simulation.")
        return True


def trigger_payload_service_call(service_suffix):
    service_name = '/inspection_payload/' + service_suffix
    try:
        rospy.wait_for_service(service_name, timeout=0.1)
        try:
            rospy.loginfo('Calling inspection service call: [' + service_name + '].')
            inspection_payload_service_call = rospy.ServiceProxy(service_name, std_srvs.srv.Trigger)
            response = inspection_payload_service_call()
            if not response.success:
                rospy.logerr("Payload could not [" + service_suffix + "].")
            return response.success
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
            return False
    except rospy.ROSException, e:
        rospy.loginfo("Did not detect inspection_payload service, assuming no payload.")
        return True


def wait_for_active_controller(controller_name, timeout=0.5):
    try:
        active_controller = rospy.wait_for_message('/anymal_highlevel_controller/notify_active_controller', ActiveController, timeout)
    except rospy.exceptions.ROSExceptio:
        rospy.logerr("Active controller not available.")
        return False
    if active_controller.name != controller_name:
        rospy.logerr("Could not find active controller [" + controller_name + "].")
        return False
    return True

def wait_for_state_estimator_status(status, timeout=2.0):
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time) < rospy.Duration(timeout):
        try:
            anymal_state = rospy.wait_for_message('/state_estimator/anymal_state', anymal_msgs.msg.AnymalState, 0.1)
        except rospy.exceptions.ROSException:
            rospy.logerr("Topic /state_estimator/anymal_state is not available within 0.1 seconds.")
            return False
        if  anymal_state.state == status:
            return True
    rospy.logerr("SE did not reach status " + str(status) + " within " + str(timeout) + " seconds.")
    return False


def get_joint_positions():
    # Get all joint positions in a dictionary
    try:
        anymal_state = get_anymal_state(0.1)
        joint_names = anymal_state.joints.name
        joint_positions = anymal_state.joints.position
        return dict(zip(joint_names, joint_positions))
    except rospy.exceptions.ROSException:
        rospy.logerr("ANYmal state not available.")
        return False


def get_anymal_state(timeout):
    try:
        anymal_state = rospy.wait_for_message("/state_estimator/anymal_state", anymal_msgs.msg.AnymalState, timeout)
        return anymal_state
    except rospy.exceptions.ROSException:
        rospy.logerr("ANYmal state not available.")
        return False