#rqt_highlevel_controlmanager

##Example Parameter File (mandatory)

```
# rqt_highlevel_controlmanager.yaml

controlmanager:
  service:
    switch_controller: "/locomotion_controller/switch_controller"
    switch_lowlevel_controller: "/lowlevel_controller/switch_controller"
    get_available_controllers: "/locomotion_controller/get_available_controllers"
    emergency_stop: "/locomotion_controller/emergency_stop"
    state_estimator_reset: "/state_estimator/reset"
    state_estimator_reset_here: "/state_estimator/reset_here"
```

Load this parameter file into the parameter server (without a namespace) before 
this rqt node starts.

##Modes

**Switch Modes Topic Name**

`"/" + <controller name> + "/go_to_mode"`

**Get Available Modes Topic Name**

`"/" + <controller name> + "/get_available_modes"`

