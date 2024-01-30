Everytime you open up the terminal to get the LiDar to spin, follow these steps:

To see the used USBs:

```ls -l /dev | grep ttyUSB```

Change permission of the USB:

``` sudo chmod 666 /dev/ttyUSB0 ```

Check the permissions:

```ls -l /dev | grep ttyUSB```

For our project, the source folder that has rplidar_ros installed is ```cd catkin_ws2```

Run ```catkin_make```

Run ```source devel/setup.bash```

In a `NEW TERMINAL`:

Run ```roscore```

Go back to the original terminal with the sourced environment and run ```roslauch rplidar_ros view_rplidar.launch```

The Rviz window should load with the map of the RPLiDar's surroundings. 
