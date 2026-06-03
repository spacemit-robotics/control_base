# Chassis Calibration Tool

Simple tool to send a constant forward velocity and print raw RPMsg feedback.

Build:

```sh
cd components/control/base/calibration
make
```

Run (defaults):

```sh
./chassis_calib
```

Options:

- `-c/--ctrl <path>`: RPMsg control device (default `/dev/rpmsg_ctrl0`)
- `-d/--data <path>`: RPMsg data device (default `/dev/rpmsg0`)
- `-s/--service <name>`: RPMsg service name (default `rpmsg:motor_ctrl`)
- `-w/--wheel <meters>`: wheel diameter in meters (default `0.067`)
- `-b/--base <meters>`: wheel base in meters (default `0.183`)
- `-v/--vel <m/s>`: target linear velocity (default `0.5`)

Press Ctrl+C to stop. The program will print every raw message received on the data device.
