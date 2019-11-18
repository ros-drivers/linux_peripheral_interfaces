#
# Copyright (c) 2019, Eurotec, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

from nose.tools import raises

from wireless_monitor.util import execute_command


def test_execute_command():
    assert "hi" == execute_command(["echo", "hi"])


@raises(RuntimeError)
def test_execute_invalid_command():
    execute_command(["--&*(", "hi"])
