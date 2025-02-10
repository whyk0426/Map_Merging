# Map Merging Techniques in Multi-robot Environments-ROS2
## Algorithm

## How to customize
The Merge Map ROS2 pkg can be customized and launched in your own 

```python
Node(
            package='merge_map',
            executable='merge_map',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ("/map0", "/Lima/map"),
                #("/map1", "/Alpha/map"),
                ("/map1", "/Romeo/map"),
                ],
        )
```

## Examples
[Example1]
![result0](https://github.com/user-attachments/assets/53397a86-f477-487e-958e-824ef70ec7a4)
[Example2]
![result1](https://github.com/user-attachments/assets/240e6fa8-426b-4871-9b8d-b9d639c5d859)
