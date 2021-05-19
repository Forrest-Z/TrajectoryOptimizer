# Trajectory Optimizer module 
This module will take the decision targets and output a smooth and collision-free trajectory 

# input
``` 
1. std::shared_ptr<zjlmap::Map> map_data 
2. std::vector<localPlanner::Box2d> obstacles 
3. localPlanner::Box2d robot
4. std::vector<localPlanner::TargetNode> targets
```

# output
``` 
1. std::vector<TrajectoryPoint> planning_trajectory_data
```


# steps
``` 
1. git clone ssh://git@10.0.105.204:2022/steve/trajectory_optimizer.git
2. cd trajectory_optimizer 
3. mkdir build && cd build 
4. cmake .. && make 
5. cd .. 
6. export LD_LIBRARY_PATH=${替换成项目绝对地址}/deps/libdrive/lib/
7. ./planning 
```
# lib path  
```
./libs/
```
