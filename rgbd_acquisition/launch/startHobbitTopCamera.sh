<launch>   
    <!-- name of the "camera" -->
    <arg name="camera" value="headcam"/> 
    <!-- name of the root frame -->
    <arg name="frame" value="frame"/> 

    <!-- Virtual Baseline for emulating disparity data , please note that disparity is a bad thing to use since we are using an active depth sensor and already have an observation in mm and meters , also please note that value given here should be in millimeters since the disparity nodelet is connected straight to the depth_registered topic which is on millimeters 
         The value -43.186 = -0.075 * 575.815735 due to a bug (?) on ROS
         see : https://github.com/ros-perception/image_pipeline/blob/hydro-devel/depth_image_proc/src/nodelets/disparity.cpp#L136
         so we are multiplying * fX until it is fixed (?)
-->

    <arg name="virtual_baseline" value="-43.186" type="double" />

 
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

    <!--  This is the kinect camera address in the hobbit PT1 I use in Vienna 3@6 -->
    <arg name="device_id" value="freenect://0"/> 
 
 


    <node name="rgbd_acquisition" pkg="rgbd_acquisition" type="run_it.sh" required="true" output="screen"> 
      <param name="camera" value="$(arg camera)" />
      <param name="device_id" value="$(arg device_id)" /> 
      <param name="virtual_baseline" value="$(arg virtual_baseline)" />   
      <param name="frame" value="$(arg frame)" /> 
      <param name="useSkeleton" value="1" />  
    </node>  

   <node pkg="nodelet" type="nodelet" name="rgbd_nodelet_manager" ns="$(arg camera)" args="manager" output="screen"/>

    <!-- The depth image is already rectified and registered to the camera optical frame, but stored in mm; convert it to meters  -->
    <node pkg="nodelet" type="nodelet" name="metric_rect" ns="$(arg camera)" args="load depth_image_proc/convert_metric rgbd_nodelet_manager --no-bond">
     <remap from="image_raw" to="/$(arg camera)/depth_registered/image_rect"/>
     <remap from="image" to="/$(arg camera)/depth_registered/image_rect_m"/>
    </node>



    <!-- Produce a disparity image  -->
    <node pkg="nodelet" type="nodelet" name="disparityBroadcaster" ns="$(arg camera)" args="load depth_image_proc/disparity rgbd_nodelet_manager --no-bond" output="screen">
     <remap from="left/image_rect" to="/$(arg camera)/depth_registered/image_rect_m"/>
     <remap from="right/camera_info" to="/$(arg camera)/rgb/camera_info"/>
     <remap from="left/disparity" to="/$(arg camera)/depth_registered/disparity"/>
     <param name="min_range" value="0.5" />
     <param name="max_range" value="4.0" />
    </node>



   <!--  Produce point cloud ( rosrun pcl_ros pointcloud_to_pcd input:=headcam/depth_registered/points/ ) to dump it -->
   <node pkg="nodelet" type="nodelet" name="cloudify" ns="$(arg camera)" args="load depth_image_proc/point_cloud_xyzrgb rgbd_nodelet_manager --no-bond" output="screen">
   <remap from="depth_registered/image_rect" to="/$(arg camera)/depth_registered/image_rect_m"/>
   <remap from="depth_registered/points" to="/$(arg camera)/depth_registered/points"/>
   <remap from="rgb/image_rect_color" to="/$(arg camera)/rgb/image_rect_color"/>
   <remap from="rgb/camera_info" to="/$(arg camera)/rgb/camera_info"/> 
  </node>
  
 
<!-- ONE could also add those
  <arg name="pi/2" value="1.5707963267948966" />
  <arg name="optical_rotate" value="0 0 0 -$(arg pi/2) 0 -$(arg pi/2)" />

  <node pkg="tf" type="static_transform_publisher" name="$(arg camera)_base_link"
        args="0 -0.02 0 0 0 0 $(arg camera)_link $(arg camera)_depth_frame 100" />
  <node pkg="tf" type="static_transform_publisher" name="$(arg camera)_base_link1"
        args="0 -0.045 0 0 0 0 $(arg camera)_link $(arg camera)_rgb_frame 100" />
  <node pkg="tf" type="static_transform_publisher" name="$(arg camera)_base_link2"
        args="$(arg optical_rotate) $(arg camera)_depth_frame $(arg camera)_depth_optical_frame 100" />
  <node pkg="tf" type="static_transform_publisher" name="$(arg camera)_base_link3"
        args="$(arg optical_rotate) $(arg camera)_rgb_frame $(arg camera)_rgb_optical_frame 100" /> 
-->



</launch>

