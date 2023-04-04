import pytest
from functools import partial

from free_gait import ActionBase
from free_gait import ActionState

def feedback(f):
    f[0] = "Done"

def test_feedback():
    f = ["None"]
    action = ActionBase(None)
    action.register_callback(partial(feedback, f), None)
    action.set_state(ActionState.ACTIVE)
    assert f[0] == "Done","Feedback was not executed."
