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
