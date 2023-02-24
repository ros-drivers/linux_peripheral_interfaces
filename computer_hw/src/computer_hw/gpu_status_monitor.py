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

from computer_status_msgs.msg import GPUStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rospy
import traceback

from computer_hw.gpu_util import GPUStatusHandler


class GpuMonitor(object):
    def __init__(self, stat_handler_class):
        """
        @param stat_handler_class: Class object that is to be delgated to return
            GPU status. E.g. computer_hw.nvidia_util.Nvidia_GPU_Stat
        @type stat_handler_class: computer_hw.gpu_util.GPUStatusHandler
        """
        # Instantiating GPU status handler.
        self._gpu_status_handler = stat_handler_class()
        if not isinstance(self._gpu_status_handler, GPUStatusHandler):
            raise TypeError("GPU status handler passed '{}' is not compatible. This class needs a derived class of {}".format(
                stat_handler_class, GPUStatusHandler))
        self._pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        self._gpu_pub = rospy.Publisher('gpu_status', GPUStatus, queue_size=10)

    def gpu_status_to_diag(self, gpu_stat):
        stat = DiagnosticStatus()
        stat.name = 'GPU Status'
        stat.message = 'OK'
        stat.level = DiagnosticStatus.OK
        stat.hardware_id = gpu_stat.pci_device_id

        stat.values.append(KeyValue(key='Product Name',         value = gpu_stat.product_name))
        stat.values.append(KeyValue(key='PCI Device/Vendor ID', value = gpu_stat.pci_device_id))
        stat.values.append(KeyValue(key='PCI Location ID',      value = gpu_stat.pci_location))
        stat.values.append(KeyValue(key='Display',              value = gpu_stat.display))
        stat.values.append(KeyValue(key='Driver Version',       value = gpu_stat.driver_version))
        stat.values.append(KeyValue(key='Temperature (C)',      value = '%.0f' % gpu_stat.temperature))
        stat.values.append(KeyValue(key='Fan Speed (RPM)',      value = '%.0f' % GPUStatusHandler.rads_to_rpm(gpu_stat.fan_speed)))
        stat.values.append(KeyValue(key='Usage (%)',            value = '%.0f' % gpu_stat.gpu_usage))
        stat.values.append(KeyValue(key='Memory (%)',           value = '%.0f' % gpu_stat.memory_usage))

        # Check for valid data
        if not gpu_stat.product_name or not gpu_stat.pci_device_id:
            stat.level = DiagnosticStatus.ERROR
            stat.message = 'No Device Data'
            return stat

        # Check load
        if gpu_stat.gpu_usage > 98:
            stat.level = max(stat.level, DiagnosticStatus.WARN)
            stat.message = 'High Load'

        # Check thresholds
        if gpu_stat.temperature > 90:
            stat.level = max(stat.level, DiagnosticStatus.WARN)
            stat.message = 'High Temperature'
        if gpu_stat.temperature > 95:
            stat.level = max(stat.level, DiagnosticStatus.ERROR)
            stat.message = 'Temperature Alarm'

        # Check fan
        if gpu_stat.fan_speed == 0:
            stat.level = max(stat.level, DiagnosticStatus.ERROR)
            stat.message = 'No Fan Speed'
        return stat

    def pub_status(self):
        stat = DiagnosticStatus()
        gpu_stat = None
        try:
            _non_ros_gpu_stat = self._gpu_status_handler.get_gpu_status()
            gpu_stat = self._convert_output(_non_ros_gpu_stat)
            stat = self.gpu_status_to_diag(gpu_stat)
            rospy.logdebug("gpu_stat: {}\n".format(gpu_stat))
        except AttributeError as e:
            rospy.logerr('Unable to process GPU status as getting GPU status with proprietary command failed : {}'.format(str(e)))
        except Exception as e:
            rospy.logerr('Unable to process GPU status: {}'.format(str(e)))
            rospy.logerr(traceback.format_exc())

        gpu_stat.header.stamp = rospy.get_rostime()

        array = DiagnosticArray()
        array.header.stamp = rospy.get_rostime()

        array.status = [ stat ]

        self._pub.publish(array)
        self._gpu_pub.publish(gpu_stat)

    def _convert_output(self, gpu_stat_proprietary):
        """
        @param gpu_stat_proprietary: 
        @rtype computer_status_msgs.GPUStatus
        """
        gpu_stat = GPUStatus()
        gpu_stat.product_name   = gpu_stat_proprietary.product_name
        gpu_stat.pci_device_id  = gpu_stat_proprietary.pci_device_id
        gpu_stat.pci_location   = gpu_stat_proprietary.pci_location
        gpu_stat.display        = gpu_stat_proprietary.display
        gpu_stat.driver_version = gpu_stat_proprietary.driver_version
        gpu_stat.temperature = gpu_stat_proprietary.temperature
        gpu_stat.fan_speed = gpu_stat_proprietary.fan_speed
        gpu_stat.gpu_usage = gpu_stat_proprietary.gpu_usage
        gpu_stat.memory_usage = gpu_stat_proprietary.memory_usage
        return gpu_stat

    def run(self):
        my_rate = rospy.Rate(rospy.get_param("gpu_monitor_rate", 1.0))
        while not rospy.is_shutdown():
            self.pub_status()
            my_rate.sleep()
