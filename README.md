# slam-workshop

## Create docker image

Run the following command in the root of this repository.

```bash
docker build -t slam-workshop-base .
```

Then update URL and TAG in .env

```bash
URL=slam-workshop-base
TAG=latest
```

## Build

Run the following command in the root of this repository.

```bash
tools/build.py
```

## Launch nodes

Run the following command in the root of this repository with mode you want to use.

```bash
# (odometry|mapping|localization)
tools/start_slam_image.sh mapping
```

The above command launches all the containers.
If you want to launch each container separately, specify service.

```bash
tools/start_slam_image.sh mapping (base|visualize)
```

## Record rosbag

```bash
tools/record.py testbag
```

## Check rosbag

```bash
tools/check_rosbag.py testbag
```

## Play rosbag

Put rosbag file in `resources` directory

Then run the following command with directory name (e.g. ros2\_localization\_bag)

```bash
tools/play_bag.py testbag
```

## Save map (in mapping mode)

Run the following command with directory name (e.g. testmap)

```bash
tools/save_map.py new_testmap
```

## Use saved map (in localization mode)

Edit .env and set the directory name specified in saving map
```bash
TARGET_MAP_DIRECTORY = "new_testmap"
```

## Trigger global localization (in localization mode)

Run the following command
```bash
tools/trigger_global_localization
```
