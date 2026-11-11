## Overview

A ROS node for using libsensors to provide diagnostics information about
the sensors (CPU temp, GPU temp, fan speed, voltage, etc) on a computer
system.

## Nodes

### libsensors_monitor

Node to publish sensor information from a computer

#### Published Topics

- `/diagnostics`
  ([diagnostic_msgs/msg/DiagnosticArray](https://docs.ros.org/en/rolling/p/diagnostic_msgs/msg/DiagnosticArray.html)):
  Diagnostic information about the computer.

#### Parameters

- `~ignore_sensors` (`string[]`): An array of sensors to ignore (the
  names of the sensors)
