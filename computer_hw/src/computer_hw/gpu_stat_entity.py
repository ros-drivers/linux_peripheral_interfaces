#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# @summary: Utilities for nvidia. This file is agnostic from framework e.g. ROS.


class GPU_Stat(object):
    """
    @summary Entity class to contain GPU status.
    @note: The format of this class is NOT dependent on any framework e.g. ROS
    """
    _product_name = ""
    _pci_device_id = ""
    _pci_location = ""
    _display = ""
    _driver_version = ""
    # TODO
    _fan_speed = 0.0
    _gpu_usage = 0
    _memory_usage = 0
    _temperature = 0

    def __init__(self):
        pass
        
    @property
    def fan_speed(self):
        return self._fan_speed

    @fan_speed.setter
    def fan_speed(self, v):
        self._fan_speed = v

    @property
    def gpu_usage(self):
        return self._gpu_usage

    @gpu_usage.setter
    def gpu_usage(self, v):
        self._gpu_usage = v

    @property
    def memory_usage(self):
        return self._memory_usage

    @memory_usage.setter
    def memory_usage(self, v):
        self._memory_usage = v

    @property
    def temperature(self):
        return self._temperature

    @temperature.setter
    def temperature(self, v):
        self._temperature = v

    @property
    def product_name(self):
        return self._product_name

    @product_name.setter
    def product_name(self, v):
        self._product_name = v

    @property
    def pci_device_id(self):
        return self._pci_device_id

    @pci_device_id.setter
    def pci_device_id(self, v):
        self._pci_device_id = v

    @property
    def pci_location(self):
        return self._pci_location

    @pci_location.setter
    def pci_location(self, v):
        self._pci_location = v

    @property
    def display(self):
        return self._display

    @display.setter
    def display(self, v):
        self._display = v

    @property
    def driver_version(self):
        return self._driver_version

    @driver_version.setter
    def driver_version(self, v):
        self._driver_version = v
