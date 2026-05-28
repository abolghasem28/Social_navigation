#!/bin/bash

# Define the target nodes
NODES=("/local_costmap/local_costmap" "/global_costmap/global_costmap")

for NODE in "${NODES[@]}"; do
    echo "--- Configuring $NODE ---"

    # 1. Update the list of sources to include the new one
    # Note: We keep 'scan' and add 'social_obstacles'
    # 2. Inject the sub-parameters explicitly
# We use the '--type' flag to ensure the node recognizes the data format
ros2 param set $NODE obstacle_layer.social_obstacles.topic "/social_obstacles"
ros2 param set $NODE obstacle_layer.social_obstacles.data_type "PointCloud2"
ros2 param set $NODE obstacle_layer.social_obstacles.marking True
ros2 param set $NODE obstacle_layer.social_obstacles.clearing False
ros2 param set $NODE obstacle_layer.social_obstacles.max_obstacle_height 3.0
ros2 param set $NODE obstacle_layer.social_obstacles.min_obstacle_height 0.0
ros2 param set $NODE obstacle_layer.social_obstacles.obstacle_max_range 10.0

echo "Injection attempt finished. Verifying..."
ros2 param get $NODE obstacle_layer.social_obstacles.topic

    echo "Finished updating $NODE"
done

echo "--- Injection Complete check it! ---"
# ros2 param dump /local_costmap/local_costmap | grep social_obstacles -A 10
# ros2 param dump /global_costmap/global_costmap | grep social_obstacles -A 10
