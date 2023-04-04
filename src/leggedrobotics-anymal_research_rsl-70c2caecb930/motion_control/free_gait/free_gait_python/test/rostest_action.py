import pytest
import rospy
import threading
import time

from free_gait import ActionBase
from free_gait import ActionState
from free_gait import WaitForState

rospy.init_node('rostest_action', anonymous=True)

def set_state(action):
    time.sleep(0.2)
    action.set_state(ActionState.ERROR)

def test_wait_for_state():
    action = ActionBase(None)
    wait_for_state = WaitForState(action, ActionState.ERROR, rospy.Duration(1))
    t = threading.Thread(target=set_state, args=(action,))
    t.start()
    wait_for_state.wait()
    t.join()
    assert action.state == ActionState.ERROR,"State error was not set."
