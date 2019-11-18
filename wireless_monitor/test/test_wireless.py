#
# Copyright (c) 2019, Eurotec, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import random
from nose.tools import raises

from wireless_monitor.wireless import get_wifi_ifname, get_iwconfig_dict_and_link_quality, get_wifi_ifnames, NetCount

wifi_ifnames = get_wifi_ifnames()


def test_wifi_ifnames():
    if wifi_ifnames:
        assert get_wifi_ifname() in wifi_ifnames

        wifi_ifname = random.choice(wifi_ifnames)
        assert get_wifi_ifname(wifi_ifname) == wifi_ifname

        try:
            get_wifi_ifname("lan0")
        except Exception as e:
            assert isinstance(e, ValueError)
    else:
        try:
            get_wifi_ifname()
        except Exception as e:
            assert isinstance(e, RuntimeError)


def test_iwconfig():
    if wifi_ifnames:
        iwconfig_dict, quality = get_iwconfig_dict_and_link_quality(get_wifi_ifname())
        assert isinstance(iwconfig_dict, dict)
        assert quality is None or isinstance(quality, float)


@raises(RuntimeError)
def test_invalid_iwconfig():
    get_iwconfig_dict_and_link_quality("lan0")


def test_net_count():
    if wifi_ifnames:
        nc = NetCount(get_wifi_ifname())
        assert "Download rate [KB/s]" not in nc.get_net_count_dict()
        assert "Download rate [KB/s]" in nc.get_net_count_dict()


@raises(RuntimeError)
def test_invalid_net_count():
    nc = NetCount("invalid_interface")
    nc.get_net_count_dict()
