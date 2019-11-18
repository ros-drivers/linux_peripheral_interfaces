#
# Copyright (c) 2019, Eurotec, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import os
import psutil
import re
import time

from .util import execute_command


def get_wifi_ifnames():
    """
    Get a list of wifi interface names
    :return: List of interface names
    """
    return [ifname for ifname in os.listdir('/sys/class/net/') if ifname[:2] == "wl"]


def get_wifi_ifname(wifi_ifname_preference=None):
    """
    Get wifi interface name with an optional preference

    If the preference is specified, the preference will be returned if it exists
    If the preference is NOT specified, the first wireless adapter will be returned

    :param wifi_ifname_preference: Optiona wifi ifname prefence e.g. wlan0
    :return: The wifi interface name
    :raises RuntimeError if there are no wireless devices available
    :raises ValueError if there are wireless devices available but the preference does not exist
    """
    wifi_ifnames = get_wifi_ifnames()
    if not wifi_ifnames:
        raise RuntimeError("No wifi ifnames found")
    if wifi_ifname_preference:
        if wifi_ifname_preference in wifi_ifnames:
            return wifi_ifname_preference
        raise ValueError(
            "Wifi ifname preference {} not in wifi ifnames {}".format(wifi_ifname_preference, wifi_ifnames))
    return wifi_ifnames[0]


def _parse_iwconfig_output(output):
    result = {}
    items = [e.strip() for e in output.replace('\n', '  ').split('  ') if ":" in e or "=" in e]
    for item in items:
        key, value = re.search('(.+?)[:=](.+)', item).groups()
        result[key] = value.strip().strip("\"")
    return result


def get_iwconfig_dict_and_link_quality(ifname):
    """
    Get the iwconfig status dictionary + link quality of an interface
    :param ifname: Wireless interface name
    :return: iwconfig status dictionary + link quality
    :raises: RuntimeError if the iwconfig information cannot be obtained
    """
    output = execute_command('iwconfig {}'.format(ifname), shell=True)
    iwconfig_dict = _parse_iwconfig_output(output)

    link_quality = None
    if 'Link Quality' in iwconfig_dict:
        numerator, denominator = iwconfig_dict['Link Quality'].split("/")
        link_quality = float(numerator) / float(denominator)

    return iwconfig_dict, link_quality


class NetCount(object):
    def __init__(self, ifname):
        """
        Net counter that monitors traffic
        :param ifname: Interface name
        """
        self._ifname = ifname
        self._last_count = None

    def get_net_count_dict(self):
        """
        Get the net count + rate for the specified interface
        :return: Net count dictionary
        :raises RuntimeError if getting counters fails
        """
        def _rate(current, last, dt):
            return ((current - last) / 1e3) / dt

        now = time.time()
        counts = psutil.net_io_counters(pernic=True)
        if self._ifname not in counts:
            raise RuntimeError("Could not get net IO counter for interface {}".format(self._ifname))

        count = counts[self._ifname]
        result = dict(count.__dict__)

        if self._last_count:
            last_stamp, last_count = self._last_count
            dt = now - last_stamp

            if dt > 0:
                result["Download rate [KB/s]"] = "{:.3f}".format(_rate(count.bytes_recv, last_count.bytes_recv, dt))
                result["Upload rate [KB/s]"] = "{:.3f}".format(_rate(count.bytes_sent, last_count.bytes_sent, dt))

        self._last_count = now, count
        return result
