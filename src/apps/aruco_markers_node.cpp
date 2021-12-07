/*
 * @Descripttion: 
 * @version: 
 * @Author: pifan
 * @Date: 2021-03-01 19:36:23
 * @LastEditors: pifan
 * @LastEditTime: 2021-03-24 15:44:32
 */
#include "aruco_markers/aruco_markers.hpp"
//#include "glog/logging.h"
#include "global_defination/global_defination.h"

using namespace aruco_markers;
using namespace agv_navigation;

int main(int argc, char **argv)
{
    // 这个是记录日志的相关代码
   // google::InitGoogleLogging(argv[0]);
    //FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    //FLAGS_alsologtostderr = true;
    //FLAGS_colorlogtostderr = true;

    ros::init(argc, argv, "aruco_markers");
    ros::NodeHandle nh;
    std::shared_ptr<ArcucoMarkersInterFace> arucoMarkersPtr_ = std::make_shared<ArcucoMarkersInterFace>(nh);
    return 0;
}