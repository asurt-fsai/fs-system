# asurt_msgs
This package is for definition of all our custom ROS messages used in the autonomous system.

## Current Messages

| Message Name | Purpose |
|---|---|
| Landmark  | Contains the location, color, and (optionally) the ID of a cone |
| LandmarkArray | Contains a header and a list of Landmark |
| NodeStatus  | Published as a heartbeat by all nodes to show their status (running, waiting, etc.) |
