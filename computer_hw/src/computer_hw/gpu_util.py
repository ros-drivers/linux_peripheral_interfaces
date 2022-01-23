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

##\author Kevin Watts

from __future__ import division

from computer_status_msgs.msg import GPUStatus
import logging
import math
import subprocess

from computer_hw.gpu_stat_entity import GPU_Stat

class GPUStatusHandler(object):
    """
    @summary: Utilities for nvidia. This file is agnostic from framework e.g. ROS.
    """
    _MAX_FAN_RPM = 4500

    @property
    def max_fan_rpm(self):
        return self._MAX_FAN_RPM

    @max_fan_rpm.setter
    def max_fan_rpm(self, v):
        self._MAX_FAN_RPM = v

    @staticmethod
    def rads_to_rpm(rads):
        return rads / (2 * math.pi) * 60

    @staticmethod
    def rpm_to_rads(rpm):
        return rpm * (2 * math.pi) / 60

    @staticmethod
    def get_raw_gpu_status():
        """Needs implemented in the derived class"""
        raise NotImplemented()

    @staticmethod
    def _find_val(output, word):
        lines = output.split('\n')
        for line in lines:
            tple = line.split(':')
            if not len(tple) > 1:
                continue

            name = tple[0].strip()
            val = ':'.join(tple[1:]).strip()

            if not name.lower() == word.lower():
                continue

            return val.strip()

        return ''

    def convert_proprietary_out(self, proprietary_output_raw):
        """
        @summary: Parse Nvidia's SMI tool output and returns in a more
            programming friendly format.
        @param proprietary_output_raw: str of shell command output i.e. output of
            'get_raw_gpu_status' method.
        @return File: gpu_stat_entity.GPU_Stat instance
        @raise AttributeError: When 'proprietary_output_raw' is not in an
            expected form.
        """
        if not proprietary_output_raw:
            raise AttributeError("Input proprietary data is empty. Can't convert")

        gpu_stat = GPU_Stat()

        gpu_stat.product_name = GPUStatusHandler._find_val(proprietary_output_raw, 'Product Name')
        gpu_stat.pci_device_id = GPUStatusHandler._find_val(proprietary_output_raw, 'PCI Device/Vendor ID')
        gpu_stat.pci_location = GPUStatusHandler._find_val(proprietary_output_raw, 'PCI Location ID') 
        gpu_stat.display = GPUStatusHandler._find_val(proprietary_output_raw, 'Display')
        gpu_stat.driver_version = GPUStatusHandler._find_val(proprietary_output_raw, 'Driver Version')

        TEMPERATURE_QUERIES = ["Temperature", "GPU Current Temp"]
        for query in TEMPERATURE_QUERIES:
            temp_str = GPUStatusHandler._find_val(proprietary_output_raw, query)
            if temp_str:
                temp, units = temp_str.split()
                gpu_stat.temperature = int(temp)
                break

        fan_str = GPUStatusHandler._find_val(proprietary_output_raw, 'Fan Speed')
        if fan_str:
            # Fan speed in RPM
            fan_spd = float(fan_str.strip('\%').strip()) * 0.01 * self.max_fan_rpm
            # Convert fan speed to Hz
            gpu_stat.fan_speed = GPUStatusHandler.rpm_to_rads(fan_spd)

        usage_str = GPUStatusHandler._find_val(proprietary_output_raw, 'GPU')
        if usage_str:
            usage = usage_str.strip('\%').strip()
            gpu_stat.gpu_usage = int(usage)

        mem_str = GPUStatusHandler._find_val(proprietary_output_raw, 'Memory')
        if mem_str:
            mem = mem_str.strip('\%').strip()
            gpu_stat.memory_usage = int(mem)

        return gpu_stat

    def get_gpu_status(self):
        """
        @summary: Get GPU status and return in an instance.
        @return GPU_Stat instance
        @raise AttributeError: When 'proprietary_output' is not in an
            expected form.
        """
        raw_output = self.get_raw_gpu_status()
        return self.convert_proprietary_out(raw_output)
