# computer_hw

## Development history
`computer_hw` package was originally made in [pr2_robot](https://github.com/PR2/pr2_robot) repository. See discussion [pr2_common#286](https://github.com/PR2/pr2_common/issues/286) for the migration.

## Installation tips
In order for ipmitool to work on computers with a BMC, the following line needs to appear in `/etc/sudoers`:

```
ALL ALL=NOPASSWD: /usr/bin/ipmitool sdr type Temperature
```

## Usage / Operation
TBD

EoF

