#
# Copyright (c) 2019, Eurotec, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import subprocess


def execute_command(cmd, shell=False, cwd=None):
    """
    Execute a command and return the stripped output
    :param cmd: The command (list) to run
    :param shell: Whether to use the shell executor
    :param cwd: Current working directory
    :return: Stripped output
    :raises RuntimeError
    """
    try:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=shell, cwd=cwd)
    except OSError as e:
        raise RuntimeError("Could not execute command {}: {}".format(cmd, e))

    stdout, stderr = p.communicate()

    if p.returncode != 0:
        raise RuntimeError("Could not execute command {}".format(cmd))
    return stdout.strip()
