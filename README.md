# ribbon_bridge_measurement
Ribbon Bridge Measurement project

### launch ribbon_bridge_measurement (LSD version)

```bash
$ roslaunch ribbon_bridge_measurement ribbon_bridge_lsd.launch  <- only measurement node
$ roslaunch ribbon_bridge_measurement ribbon_bridge_lsd_with_rviz.launch  <- with rviz
```

### launch ribbon_bridge_measurement (Harris-Corner version)

```bash
$ roslaunch ribbon_bridge_measurement ribbon_bridge_lsd.launch  <- only measurement node
$ roslaunch ribbon_bridge_measurement ribbon_bridge_lsd_with_rviz.launch  <- with rviz
```


### Publications:
 * /boat_result [sensor_msgs/Image]
 * /detect_boat_polygon [jsk_recognition_msgs/PolygonArray]
 * /overlay_text [jsk_rviz_plugins/OverlayText]
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]

### Subscriptions: 
 * /boat_measurement_contrl [unknown type]
 * /darknet_ros/bounding_boxes [unknown type]
 * /usb_cam/image_raw [unknown type]
