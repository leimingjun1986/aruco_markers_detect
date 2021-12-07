#include   <aruco_markers/aruco_markers_nodelet.hpp>
 #include   <pluginlib/class_list_macros.h>

 using  namespace   aruco_markers;

aruco_markers_nodelet::aruco_markers_nodelet()
{
    ;
}

aruco_markers_nodelet::~aruco_markers_nodelet(){
    ;
}

//nodelet 动态加载入口
void aruco_markers_nodelet::onInit()
{
  ros::NodeHandle nh=getNodeHandle();
  ros::NodeHandle nh_priv=getPrivateNodeHandle();
  arcuco_marker_interface_ptr.reset(new  ArcucoMarkersInterFace(nh));    
}

PLUGINLIB_EXPORT_CLASS(aruco_markers::aruco_markers_nodelet, nodelet::Nodelet)