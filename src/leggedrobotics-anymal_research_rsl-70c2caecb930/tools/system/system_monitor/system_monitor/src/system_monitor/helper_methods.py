#!/usr/bin/env python

import subprocess


def get_file_content(path):
    with open(path) as file:
        return file.read().strip()


def get_command_stdout(command):
    popen = subprocess.Popen(
        command.split(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    result = popen.wait()
    (stdout, stderr) = popen.communicate()
    if result != 0:
        raise Exception('Failed to run \'' + command + '\'. Return value: ' +
                        str(result) + ', output: \'' + stderr + '\'.')
    return stdout


def get_max_diag_status_level(level1, level2):
    return max(level1, level2)
