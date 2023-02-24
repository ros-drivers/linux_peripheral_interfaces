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

import logging
import subprocess

from computer_hw.gpu_util import GPUStatusHandler


class Nvidia_GPU_Stat(GPUStatusHandler):
    def get_raw_gpu_status(self):
        """
        @summary: Relying on a command on the host 'nvidia-smi'.

            Regarding 'nvidia-smi', some people believe that it at least needs to be run by 'root'
            for the first invocation https://serverfault.com/questions/975859/nvidia-smi-must-be-run-by-root-before-it-can-be-used-by-regular-users,
            but it seems to be working without initial invocation.
        @todo: OpenQuetion-1: When this method is invoked from a container where
             nvidia-smi, which is typically available on a host, is not easily
             available. -> For docker, passing '--runtime=nvidia' enables the cmd
             from a container. Then show warning when unavailable.
        @todo: OpenQuetion-2: What if the cmd 'nvidia-smi' is not available?
        """
        p = subprocess.Popen('nvidia-smi -a', stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, shell=True)
        (o, e) = p.communicate()

        if not p.returncode == 0:
            return ''

        if not o: return ''
        logging.debug("card_out: {}".format(o))
        return o
