# Wireless monitor

## Nodes

### wireless_monitor

Monitors a given wireless interface using `psutil` and `iwconfig`.

#### Parameters

- `~interface`: Wireless interface to monitor (default=`'wlo1'`)
- `~link_quality_warning_percentage`: On what link quality a warning level will be given (default=`0.5`)
- `~rate`: Publish rate (default=`1`)
