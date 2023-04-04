#! /usr/bin/env python

import roslib
roslib.load_manifest('anymal_free_gait_action_loader_adapter')
import rospy
import free_gait_msgs.srv
import anymal_msgs.srv

class ActionLoaderAdapter:

    def __init__(self):
        self.collection = rospy.get_param('~collection')
        self.list_actions_service_name = '/free_gait_action_loader/list_actions'
        self.send_action_service_name = '/free_gait_action_loader/send_action'
        rospy.wait_for_service(self.list_actions_service_name)
        rospy.wait_for_service(self.send_action_service_name)

    def list_actions(self, anymal_request):
        try:
            list_actions = rospy.ServiceProxy(self.list_actions_service_name, free_gait_msgs.srv.GetActions)
            request = free_gait_msgs.srv.GetActionsRequest()
            request.collection_id = self.collection
            response = list_actions(request)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        anymal_response = anymal_msgs.srv.GetAvailableControllersResponse()
        for action_id in response.actions:
            anymal_response.available_controllers.append(action_id.id)
        return anymal_response

    def send_action(self, anymal_request):
        anymal_response = anymal_msgs.srv.SwitchControllerResponse()
        try:
            send_action = rospy.ServiceProxy(self.send_action_service_name, free_gait_msgs.srv.SendAction)
            request = free_gait_msgs.srv.SendActionRequest()
            request.goal.action_id = anymal_request.name
            response = send_action(request)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            anymal_response.status = anymal_response.STATUS_ERROR
            return anymal_response
        anymal_response.status = response.result.status
        return anymal_response


if __name__ == '__main__':
    try:
        rospy.init_node('free_gait_action_loader_adapter')
        adapter = ActionLoaderAdapter()
        rospy.Service('/free_gait/get_available_modes', anymal_msgs.srv.GetAvailableControllers, adapter.list_actions)
        rospy.Service('/free_gait/go_to_mode', anymal_msgs.srv.SwitchController, adapter.send_action)
        rospy.loginfo("Ready to load actions from service call (adapter).")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
