^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package laptop_battery_monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2022-08-27)
------------------
* Support Python3 for noetic (`#19 <https://github.com/ros-drivers/linux_peripheral_interfaces/issues/19>`_)
    These changes allow the node to work with noetic, ubuntu 20.04, with python3

* Contributors: Jeremy Fix, Kei Okada

0.2.1 (2019-11-16)
------------------
* Merge pull request `#10 <https://github.com/ros-drivers/linux_peripheral_interfaces/issues/10>`_ from kdhansen/kinetic
  Fix for systems without a power_now file when unplugged.
* Merge pull request `#12 <https://github.com/ros-drivers/linux_peripheral_interfaces/issues/12>`_ from ernestmc/add_batt_diag
  Add battery metrics to diagnostics
* Merge branch 'kinetic' into add_batt_diag
* Merge pull request `#15 <https://github.com/ros-drivers/linux_peripheral_interfaces/issues/15>`_ from k-okada/fix_maintaner
  Change maintainer to ROS Orphaned Package Maintainers
* Change maintainer to ROS Orphaned Package Maintainers
* Add battery charging state.
* Add battery percentage.
* Fix for systems without a power_now file when unplugged.
* Contributors: Ernesto Corbellini, Karl D. Hansen, Kei Okada

0.1.4 (2016-05-11)
------------------
* Switch to the new sensor_msgs/BatteryState message

0.1.1 (2014-11-18)
------------------
* Added using hostname as diagnostic hardware id for laptop battery monitor
* Contributors: Mitchell Wills

0.1.0 (2014-08-14)
------------------
* update changelog
* updatting version
* update changelog
* cleanup and now it uses smart_battery_msgs
* rename the package and add metapackage
* Contributors: Jihoon Lee
