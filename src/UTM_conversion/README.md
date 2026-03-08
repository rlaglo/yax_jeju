# Waypoint_Data_Conversion
## A ROS2 package for data conversion between latitude-longitude and UTM (Cartesian) coordinates

This package (named **latlong_utm**) has four nodes: **lattoutm**, **utmtolat**, **latlonpub** and **utmpub**

1. **lattoutm** takes in lat-lon coordinates (from topic2), converts them into utm coordinates and publishes them to topic1

2. **utmtolat** takes in utm coordinates (from topic1), converts them into lat-lon coordinates and publishes them to topic2

3. **latlonpub** publishes (to topic2 ) a few lat-lon coordinates for conversion into utm

4. **utmpub** publishes (to topic1) a few utm coordinates for conversion into lat-lon

utm coordinates consist of easting (x), northing (y), zone number and zone letter. For more information, check out the [github repo](https://github.com/Turbo87/utm) of the library we used for data conversion

Command to run a node:
```shell
ros2 run latlong_utm node_name
```
