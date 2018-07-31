# odrive_ros
ROS driver for the [ODrive motor driver](https://odriverobotics.com/)

This is a very basic first pass at a ROS driver. It's Python-based, so not super-fast, but it'll get you started. Maybe in time this will have an optimised C++ version, but I woudn't count on it anytime soon. ;)

It's also just a few files you add into your existing ROS workspace. Also on the todos is bundle it into a proper ROS package.

## Usage

As of July 31, the main ODrive repository doesn't support Python 2.7, so you will need to ensure the version you install has [my compatibility patch](https://github.com/madcowswe/ODrive/pull/199) (said patch is also is a bit rough around the edges, you will need to use the odrivetool with `python2 odrivetool` for instance).

In your existing ROS workspace, copy `odrive_node.py` (the actual ROS interface) and `odrive_interface.py` (exposes the ODrive with setup and drive functions, you can use this separately for testing). Then add a odrive.launch file containing:

```
    <node pkg="my_ros_workspace" type="odrive_node.py" name="odrive" output="screen" />
```

then `roslaunch <my_ros_workspace> odrive.launch` will get you on your way.

If you want to test out this driver separate from ROS, fire up a Python shell and `import odrive_interface`. You can use either ODriveInterfaceAPI (uses USB) or ODriveInterfaceSerial (requires the /dev/ttyUSBx corresponding to the ODrive as parameter).

```
    import odrive_interface
    od = odrive_interface.ODriveInterfaceAPI()
    od.setup()          # does a calibration
    od.engage()         # get ready to drive
    od.drive(1000,1000) # drive axis0 and axis1
    od.release()        # done
```

Thanks to the ODrive team for a great little bit of hardware and the active community support. Hope this helps! Feedback, issues and pull requests welcome.