
### Dependencies
1) [IXWebSocket](https://github.com/machinezone/IXWebSocket) Please follow the instructions on the source repo.
2) [json](https://github.com/nlohmann/json) Included in `src/include/ar_perception_client_pkg/nolhmann_json.hpp`.

### ROS command
`rosrun ar_perception_client_pkg ar_perception_client_pkg __name:=different_name _stream_name:="stream-name"`

### Possible stream names
```
upper-velodyne-vlp16/depth/points
forward-tis-dfm27up/color/image-raw
forward-chest-realsense-d435/color/image-rect
forward-chest-realsense-d435/left-infrared/image-rect
forward-chest-realsense-d435/right-infrared/image-rect
forward-chest-realsense-d435/depth/image-rect
forward-chest-realsense-d435/depth/points
forward-pelvis-realsense-d430/left-infrared/image-rect
forward-pelvis-realsense-d430/right-infrared/image-rect
forward-pelvis-realsense-d430/depth/image-rect
forward-pelvis-realsense-d430/depth/points
downward-pelvis-realsense-d430/left-infrared/image-rect
downward-pelvis-realsense-d430/right-infrared/image-rect
downward-pelvis-realsense-d430/depth/image-rect
downward-pelvis-realsense-d430/depth/points
backward-pelvis-realsense-d430/left-infrared/image-rect
backward-pelvis-realsense-d430/right-infrared/image-rect
backward-pelvis-realsense-d430/depth/image-rect
backward-pelvis-realsense-d430/depth/points
```

### TODO
<ol>
<li> Currently only implementing `flow-rate=none`. Are `framerate` and `request` needed? </li>
<li> Documentation.</li>
</ol>

### Recording a bag
`rosbag record /digit/upper_velodyne_vlp16/depth/points /digit/forward_chest_realsense_d435/depth/points /digit/forward_pelvis_realsense_d430/depth/points /digit/downward_pelvis_realsense_d430/depth/points /digit/backward_pelvis_realsense_d430/depth/points /tf`
