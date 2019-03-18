Start the Hovering example:
roslaunch rotors_gazebo mav_hovering_example.launch world_name:=waypoint

Install octomap_server and octomap_mapping

Edit octomap_server/launch/octomap_mapping.launch
<param name="frame_id" type="string" value="world" />
<param name="sensor_model/max_range" value="10.0" />
<remap from="cloud_in" to="/iris/pcl2_cloud" />

Start octomapping:
roslaunch octomap_server octomap_mapping.launch

Open RViz and check the occupancy map.
