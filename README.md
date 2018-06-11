# Ribbon-Bridge-Measurement
Ribbon Bridge Measurement project

### launch Ribbon-Bridge-Measurement (LSD version)

```bash
$ roslaunch Ribbon-Bridge-Measurement ribbon_bridge_lsd.launch  <- only measurement node
$ roslaunch Ribbon-Bridge-Measurement ribbon_bridge_lsd_with_rviz.launch  <- with rviz
```

### launch Ribbon-Bridge-Measurement (Harris-Corner version)

```bash
$ roslaunch Ribbon-Bridge-Measurement ribbon_bridge_lsd.launch  <- only measurement node
$ roslaunch Ribbon-Bridge-Measurement ribbon_bridge_lsd_with_rviz.launch  <- with rviz
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
