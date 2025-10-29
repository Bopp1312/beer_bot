# beer_bot
Repository for a ROS2 based navigation robot for the express purpose of delivering beers to the couch

## Taking a Scan of an enviornment
Generating a map of your enviornment is critical for navigating confidently between rooms and across various terrain, this is because the beer_bot while equipped with wheel encoders is subject to drift and there fore additional sensing modalities are nessary to ensure the robot knows where it is. 
Start by running the manual mapping launch file
`ros2 launch beer_bot manual_mapping.launch.py`
then in a new terminal run 
`ros2 run slam_toolbox online_async_launch.py`
now you can manually drive the robot around and collect mapping data which will be published on the /map topic 
Finally when you are ready to save the map in a new terminal run:
`ros2 run nav2_map_server map_saver_cli -f my_new_map

## Loading the map for localization

## Running navigation using custom navigation