# Project Structure

```text
├── compute
│   └── src
│       ├── fast_pointcloud
│       └── utils
├── docs
│   ├── internal
│   │   ├── api
│   │   ├── dev
│   │   └── _static
│   └── public
│       └── install
│           └── dependencies
├── examples
│   ├── aruco_markers
│   ├── modules
│   │   ├── lidar
│   │   ├── movement
│   │   ├── ocr
│   │   └── video
│   └── terrain_generator
│       └── resources
├── go2
│   ├── communication
│   ├── core
│   └── modules
│       ├── audio
│       ├── input
│       ├── lidar
│       ├── movement
│       ├── ocr
│       └── video
│           ├── sources
│           └── streaming
├── refs
│   ├── condarefs
│   └── ubunturefs
├── requirements
├── ros2_ws
│   └── src
│       ├── bringup
│       │   ├── config
│       │   └── launch
│       └── lidar_processor
│           ├── benchmark
│           ├── lidar_processor
│           ├── resource
│           └── test
└── scripts
```

`compute/` - Python-callable utilities written in C++.

`docs/internal/` - Markdown files for building API documentation. \
`docs/internal/api/` - Markdown files for building student-facing API documentation.

`docs/public/` - Formatted documentation for students regarding the SDK itself.

`examples/` - SDK usage examples. \
`examples/aruco_markers/` - Examples for detecting and reacting to Aruco markers. \
`examples/modules/lidar/` - Examples for working with the controller's LIDAR capabilities. \
`examples/modules/movement/` - Examples for working with the controller's movement capabilities. \
`examples/modules/ocr/` - Examples for working with the controller's OCR capabilities. \
`examples/modules/video/` - Examples for working with the controller's video capabilities. \
`examples/terrain_generator/` - Examples for working with the simulator's terrain generation capabilities.

`go2/` - Python package containing all SDK functionalities. \
`go2/communication/` - CycloneDDS configurations. \
`go2/core/` - Core SDK functionalities. \
`go2/modules/` - Implementation code for each module (robot functionality) supported by the SDK. \

`refs/condarefs/` - Captured packages for school conda environment. \
`refs/ubunturefs/` - Information about school Ubuntu device.

`requirements/` - Python dependency information needed for package installation and documentation builds.

`ros2_ws/` - Workspace for communicating with the robot via ROS2. \
`ros2_ws/src/bringup/` - Files for creating ROS2 node launch scripts. \
`ros2_ws/src/lidar_processor/` - ROS2 nodes for decoding Lidar information published by the physical robot.

`scripts/` - .sh files for miscellaneous use.
