# NDT_registration
NDT_registration for velodyne scans
  
  NDT registration algorithm that refister a new velodyne scan to the previous scans saven in "map_local"
  
  use "roslaunch ndt_localizer ndt_mapping.launch" to run the node
  
  
  
  Param in launch file:   
  range: the minimum distance for valid points. The points inside this range will not be used.
  shift: the minimum distance difference to add a new scan to the global point map. (the global point map is only used to display)
  
  map_local_length:       buffer size of the local scans
  map_terrain_length:     not used
  shift_terrain:          the minimum distance difference to add a new scan to the local scan buffer

  points_raw:             topic name of the velodyne points
