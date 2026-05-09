# Maps

Place your saved map files here (.yaml + .pgm).

To save a map after running SLAM:
  ros2 run nav2_map_server map_saver_cli -f maps/training_map

Then reference training_map.yaml in the navigation launch file.
