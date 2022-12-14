<launch>   
    <!-- name of the "camera" -->
    <arg name="camera" value="basecam"/> 
    <!-- name of the root frame -->
    <arg name="frame" value="frame"/> 

    <!-- device_id can have the following formats:
         "B00367707227042B": Use device with given serial number
         "#1"              : Use first device found
         "2@3"             : Use device on USB bus 2, address 3
	 "2@0"             : Use first device found on USB bus 2
         "0"               : Use first device enumerated by the OpenNI driver
         "1"               : Use second device enumerated by the OpenNI driver
         "Path/to/onifile/some.oni" : Use ONI file
         This can also be the serial number of the top Kinect - gathered via "lsusb -v"
         ______________________________________________________________________   
    -->
    <arg name="device_id" value="3@3"/> 
 
 
   <!--  rosrun pcl_ros pointcloud_to_pcd input:=headcam/depth_registered/points/ -->


    <node name="rgbd_acquisition" pkg="rgbd_acquisition" type="run_it.sh" required="true" output="screen"> 
      <param name="camera" value="$(arg camera)" />
      <param name="device_id" value="$(arg device_id)" /> 
      <param name="frame" value="$(arg frame)" /> 
      <param name="useSkeleton" value="0" />  
    </node>  

   <node pkg="nodelet" type="nodelet" name="nodelet_manager" ns="$(arg camera)" args="manager" output="screen"/>

    <!-- The depth image is already rectified and registered to the camera optical frame, but stored in mm; convert it to meters  -->
    <node pkg="nodelet" type="nodelet" name="metric_rect" ns="$(arg camera)" args="load depth_image_proc/convert_metric nodelet_manager --no-bond">
     <remap from="image_raw" to="/$(arg camera)/depth_registered/image_rect"/>
     <remap from="image" to="/$(arg camera)/depth_registered/image_rect_m"/>
    </node>

   <node pkg="nodelet" type="nodelet" name="cloudify" ns="$(arg camera)" args="load depth_image_proc/point_cloud_xyzrgb nodelet_manager --no-bond" output="screen">
   <remap from="depth_registered/image_rect" to="/$(arg camera)/depth_registered/image_rect_m"/>
   <remap from="depth_registered/points" to="/$(arg camera)/depth_registered/points"/>
   <remap from="rgb/image_rect_color" to="/$(arg camera)/rgb/image_rect_color"/>
   <remap from="rgb/camera_info" to="/$(arg camera)/rgb/camera_info"/> 
  </node>


</launch>

