# cse280

All the packages used in our self driving car simulation.  

## Structure
Nodes are grouped into packages with like nodes, but the whole graph is started from the launch file in `dv_master/launch/`.

```
.
├── dv_master/
|   └── launch/
|       └── master.launch
├── dv_perception/
|   ├── camera
|   ├── lidar
|   ├── odometer
|   ├── gyroscope
|   └── perception_aggregator
├── dv_mapping/
|   ├── mapping_node
|   └── planning_node
└── dv_locomotion/
    ├── locomotion_node
    ├── steering
    ├── propulsion
    └── xbox_controller
```