# object_detection_tracking

## Overview

![object detection and tracking](./overview/DT-2020-06-29_22.31.03.mkv)

## Run

```bash
$1: roscore
$2: rviz
$3: cakin_make
source ./devel/setup.bash
rosrun object_detection velo32_perception
$4: rosbag play data.bag
```

## Data

rosbag data: [baidu pan](https://pan.baidu.com/s/1mE3lrXIhxCJZ3qfk0VCqAA) (提取码: suwb )

## Reference

[1]. [Autoware Tracking](https://github.com/Autoware-AI/core_perception/tree/master/lidar_imm_ukf_pda_track)
