# Map Merging Techniques in Multi-robot Environments-ROS2
## How to use
This package is designed to merge map messages from `/map` and `/map2` topics, and publish the merged map to the `/merge_map` topic after figuring out the difference between the robots. To use this package, follow the instructions below:

 1. Copy the `merge map` folder to the `src` directory of your ROS2 workspace.
 2. Run the following command: `ros2 launch merge_map merge_map_launch.py` This command starts the map merging node and opens an RViz2 window.
 3. In the RViz2 window, you can observe the merged map. Whenever one of the maps being merged is updated, the `merge_map` node will also update the merged map accordingly.

Make sure to have ROS2 installed and properly configured in your environment before using this package. For more information, please refer to the ROS2 documentation.
## How to customize
The Merge Map ROS2 pkg can be customized and launched in your own `launch.py` file by adding the following code block. You can update `/yourMapName1` and `/youtMapName2` topics according to yoour project.
```python
Node(
            package='merge_map',
            executable='merge_map',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ("/map0", "/Lima/map"),
                ("/map1", "/Romeo/map"),
                ],
        )
```

## Examples
[Example1]
![result0](https://github.com/user-attachments/assets/53397a86-f477-487e-958e-824ef70ec7a4)
[Example2]
![result1](https://github.com/user-attachments/assets/240e6fa8-426b-4871-9b8d-b9d639c5d859)
