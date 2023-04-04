#! /usr/bin/env python

file_path = action_loader.directory + '/walk_joy.py'
execfile(file_path, globals(), globals())
action.base_height = 0.28
action.default_foot_position[Leg.LF]  = array([ 0.5,  0.23])
action.default_foot_position[Leg.RF] = array([ 0.5, -0.23])
action.default_foot_position[Leg.LH]  = array([-0.5,  0.23])
action.default_foot_position[Leg.RH] = array([-0.5, -0.23])
