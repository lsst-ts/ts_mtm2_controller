# Deployment in cRIO controller

The M2 controller cRIO code is compiled and deployed in the cRIO controller.

## Initialization File

Copy the [initialization file](../script/m2) to the `/etc/init.d/` directory.

## Use the Control System

To start the control system, do:

```bash
/etc/init.d/m2 start
```

To stop the control system, do:

```bash
/etc/init.d/m2 stop
```

To get the status of the control system, do:

```bash
/etc/init.d/m2 status
```

More usage, do:

```bash
/etc/init.d/m2 -h
```

## Note of the Failure to Run the M2 Automatically after the System Reboot

I tried to run the control system automatically with the following script that can be added by `update-rc.d` with the `defaults` argument:

```bash
#!/bin/sh

CONTROL_SYSTEM_DIR=/home/admin/github/ts_mtm2_controller
EXECUTABLE=${CONTROL_SYSTEM_DIR}/target/release/run_m2

# Sleep 120 seconds for the OS and FPGA bitfile to be ready
sleep 120

# Start the control system.
echo "Starting M2 control system. Sleep 30 seconds to allow it to boot up."

ulimit -s unlimited

nohup ${EXECUTABLE} > /dev/null 2>&1 &
# For the simulation mode, use the following command instead.
# nohup ${EXECUTABLE} -s > /dev/null 2>&1 &

sleep 30
```

However, the control system can not start up itself that I do not know the reason.
In addition, the LabVIEW runtime engine does not load the FPGA bitfile as well based on the `/var/local/natinst/log/errlog.txt`.
Therefore, at the moment, the user needs to manually start up the control system after resetting the system.
Record this trying and result here as a reference.
