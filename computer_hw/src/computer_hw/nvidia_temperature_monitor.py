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

from computer_hw.gpu_util import GPUStatusHandler


class NVidiaTempMonitor(object):
    def __init__(self):
        self._pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        self._gpu_pub = rospy.Publisher('gpu_status', GPUStatus, queue_size=10)

    def gpu_status_to_diag(gpu_stat):
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
        gpu_stat = GPUStatus()
        stat = DiagnosticStatus()
        try:
            card_out = get_gpu_status()
            gpu_stat = parse_smi_output(card_out)
            stat = gpu_status_to_diag(gpu_stat)
            rospy.loginfo("card_out: {}\ngpu_stat: {}\n".format(card_out, gpu_stat))
        except Exception as e:
            import traceback
            rospy.logerr('Unable to process nVidia GPU data')
            rospy.logerr(traceback.format_exc())

        gpu_stat.header.stamp = rospy.get_rostime()

        array = DiagnosticArray()
        array.header.stamp = rospy.get_rostime()

        array.status = [ stat ]

        self._pub.publish(array)
        self._gpu_pub.publish(gpu_stat)


def parse_smi_output(output):
    gpu_stat = GPUStatus()

    gpu_stat.product_name   = GPUStatusHandler._find_val(output, 'Product Name')
    gpu_stat.pci_device_id  = GPUStatusHandler._find_val(output, 'PCI Device/Vendor ID')
    gpu_stat.pci_location   = GPUStatusHandler._find_val(output, 'PCI Location ID')
    gpu_stat.display        = GPUStatusHandler._find_val(output, 'Display')
    gpu_stat.driver_version = GPUStatusHandler._find_val(output, 'Driver Version')

    TEMPERATURE_QUERIES = ["Temperature", "GPU Current Temp"]
    for query in TEMPERATURE_QUERIES:
        temp_str = GPUStatusHandler._find_val(output, query)
        if temp_str:
            temp = temp_str.split()[0]
            gpu_stat.temperature = int(temp)
            break

    fan_str = GPUStatusHandler._find_val(output, 'Fan Speed')
    if fan_str:
        # Fan speed in RPM
        fan_spd = float(fan_str.strip('\%').strip()) * 0.01 * GPUStatusHandler._MAX_FAN_RPM
        # Convert fan speed to Hz
        gpu_stat.fan_speed = GPUStatusHandler.rpm_to_rads(fan_spd)

    usage_str = GPUStatusHandler._find_val(output, 'GPU')
    if usage_str:
        usage = usage_str.strip('\%').strip()
        gpu_stat.gpu_usage = int(usage)
        
    mem_str = GPUStatusHandler._find_val(output, 'Memory')
    if mem_str:
        mem = mem_str.strip('\%').strip()
        gpu_stat.memory_usage = int(mem)

    return gpu_stat

