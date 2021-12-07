
#include "aruco_marker_map_build/aruco_single_mark.hpp"

using namespace cv;
using namespace aruco;
using namespace std;
using namespace chrono;
namespace aruco_markers
{
    //ArucoMarkers类构造函数
ArucoSingleMarker::ArucoSingleMarker(ros::NodeHandle& nh, ArucoSingleMarkerParam&  param):nh_(nh),
                                                nhPriv_("~"),
                                                it_(nhPriv_) 
{
    aruco_single_marke_param =  param;   //参数结构体传入
    cameraIndex_ =aruco_single_marke_param.cameraIndex_;
    ROS_INFO("cameraIndex_: %d",cameraIndex_);
    std::string   image_topic = aruco_single_marke_param.camera_topic_name;
    ROS_INFO("image_topic: %s",image_topic.c_str());
    imageSub_ = it_.subscribe( image_topic,  1,  &ArucoSingleMarker::imageCallback,  this);    //订阅图像
    ROS_INFO("camera_detect_topic_name: %s",aruco_single_marke_param.camera_detect_topic_name.c_str());
    imagePub_ = it_.advertise(aruco_single_marke_param.camera_detect_topic_name,  1);      //  检测结果图像 topic  
    ROS_INFO("topic_cameral_control: %s",aruco_single_marke_param.topic_cameral_control.c_str());
    cameraContolSub_ = nhPriv_.subscribe(aruco_single_marke_param.topic_cameral_control, 1, &ArucoSingleMarker::localizationEnableCallback, this);  //订阅相机控制信号
    ROS_INFO("topic_markers_map_pose: %s",aruco_single_marke_param.topic_markers_map_pose.c_str());
    arucoMarkersMapPosePub_ = nhPriv_.advertise<vision_msgs::ArucoMarkersMapPose>(aruco_single_marke_param.topic_markers_map_pose, 1);   //多个topic  名字不同
   
    cameraCalibrationFilePath_ = aruco_single_marke_param.camera_param_file_path +  aruco_single_marke_param.camera_param_file_name;
     ROS_INFO("cameraCalibrationFilePath_: %s",cameraCalibrationFilePath_.c_str());
    
   localizationEnable_   =  aruco_single_marke_param.localizationEnable ;
    debug_= aruco_single_marke_param.debug;

    child_frame_id=aruco_single_marke_param.child_frame_id;
    frame_id=aruco_single_marke_param.frame_id;

   //相机内参 从xml文件读取
    cameraParameters_.readFromXMLFile(cameraCalibrationFilePath_);
    //只需要传递字典名称
    markerDetector_.setDictionary("ARUCO_MIP_36h12");
   //二维码边长  以m为单位
    markerSize_ = param.markerSize;
    ROS_INFO("Camera id %d : ArucoSingleMarkers  construct  finish",aruco_single_marke_param.cameraIndex_);
}  

/**
 * @name: 
 * @brief: 
 * @Date: 2021-03-23 20:48:30
 * @param {const} YAML
 * @return {*}
 */

/**
 * @name: imageCallback
 * @brief: 图像回调函数
 * @Date: 2021-03-02 09:57:48
 * @params {*}
 * @return {*}
 */
void ArucoSingleMarker::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    /*
    if (localizationEnable_ == false)            //定位开关  通过launch文件配置     他通过/camera_control 消息  控制定位开关的开关
       {
           //ROS_INFO("camera_id  %d  imageCallback",aruco_marker_param.cameraIndex_);
           return;
       }  */
    try
    {
        std::cout<<"camera  id:"<<cameraIndex_<<"  marker detect."<<std::endl;
        auto start = system_clock::now();
        captureTimeStamp_ = msg->header.stamp;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,  sensor_msgs::image_encodings::MONO8);//RGB8;                                              
        inImage_ = cv_ptr->image;



        //对图像进行
        // 检测二维码，并且当前定位到的marker的ID提取出来
        detectMarkers(inImage_, currentMarkersIdSet_); //输入 inImage_,   输出  currentMarkersIdSet_ 二维码id集合
        // 检查二维码ID是否在正常范围内，防止出现误判   只给出了打印信息
        //checkMarkersId(markers_);    //markers_  检测到的所有二维码   在上一个函数中计算得到         没有经过与二维码库id比对过滤    
        // 对获取的二维码进行分类
             // classfiyMarkersCoordinates(currentMarkersIdSet_);   //currentMarkersIdSet_   经过过滤后的二维码id集合
        // classfiyIdWall(currentMarkersIdSet_);
        // 获取相机的在全局二维码坐标系下的位姿    重要计算函数
        if(currentMarkersIdSet_.size()>0)
        getCameraPose(markers_[0]);    //传入未过滤的二维码id及角点？
        else{
            ROS_INFO("no mark detected. markers size %d",markers_.size());
        }

        auto end = system_clock::now();
         microseconds duration = duration_cast<microseconds>(end - start);
        double computeTime = duration.count()/1000.0;
        //ROS_INFO_STREAM("aruco markers map average time: " <<  computeTime <<" ms"<<std::endl);
    }
    catch(cv_bridge::Exception& e)
    {
       ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    catch(cv::Exception& e)
    {
       ROS_ERROR("cv exception: %s", e.what());
    }
}

/**
 * @name: 
 * @brief: 相机控制的回调函数
 * @Date: 2021-03-02 18:56:00
 * @params {*}
 * @return {*}
 */

void ArucoSingleMarker::localizationEnableCallback(const vision_msgs::CameraControl &msg)
{
    uint8_t controlByte = msg.controlByte;
     (controlByte>>cameraIndex_) & 0x01  ? localizationEnable_ = true : localizationEnable_ = false;  
    std::cout<<cameraIndex_<<"    enter  cameracontrol callback"<<"  localizationEnable_:"<<localizationEnable_<<std::endl;
}


/*
bool ArucoSingleMarker::markersIdSetIntersection(const std::set<std::size_t>& setL,
                                            const std::set<std::size_t>& setR)
{
    std::set<std::size_t> setInter;
    std::set_intersection(setL.begin(),
                          setL.end(),
                          setR.begin(),
                          setR.end(), 
                          std::inserter(setInter, setInter.begin()));
    if (setInter.size()) 
        return true;
    else
        return false;  
}*/

 /**
  * @name: 
  * @brief: 区分二维码的坐标系
  * @Date: 2021-03-25 09:40:55
  * @param {const} std
  * markersIdSet： 输入的当前定位得到的二维码
  * @return {*}
  */

/**
 * @name: 
 * @brief: 检测二维码
 * @Date: 2021-03-04 11:36:06
 * @params {*}
 * @return {*}
 * @param {const} cv
 */
void ArucoSingleMarker::detectMarkers(const cv::Mat& inImage,
                                 std::set<std::size_t>& markersIdSet)    //输入图像   输出二维码id   经过了过滤
{
    markersIdSet.clear();
    // 检测二维码
    markers_ = markerDetector_.detect(inImage);    // std::vector<aruco::Marker> markers_   二维码的图像二维信息 
    // 在估计姿态的时候还会过滤一次   返回  vector<int>类型    检测到的二维码与 预先输入的 MarkerMap二维码 id做比较 
    numbersOfMarkers_  =  markers_.size();
    // 如果检查ID发现是特殊的ID，就将isSpecialId_设置为true
    // checkSpecialId(markersIndexSet);
    cv::Mat inRGBImage;
    // 将灰度图转换为RGB图
    cv::cvtColor(inImage, inRGBImage, COLOR_GRAY2RGB);    //占用了计算时间
    // 将识别的二维码结果在图像上做标记
    // 打印二维码ID，并对图像做标记
    for (int  idx =0;idx<numbersOfMarkers_;idx++)
    { 
        if (debug_)    
        {
            // 打印二维码ID
            if (idx == 0)
            std::cout  <<"numbersOfMarkers:" <<numbersOfMarkers_<< std::endl;
            std::cout  << "ID: " << markers_[idx].id <<"     " ;
            /*
            if (idx == numbersOfMarkers_-1)
            {
                std::cout  << "--------"<< std::endl;
                std::cout  <<std::endl;
                std::cout  <<std::endl;
            }*/
            markers_[idx].draw(inRGBImage, Scalar(0, 0, 255), 1);
        }
        markersIdSet.insert(markers_[idx].id);
    }
     std::cout  <<std::endl;
    if (debug_)
    {
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = ros::Time::now();
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inRGBImage;
        imagePub_.publish(out_msg.toImageMsg());
    }

    ROS_INFO("detectMarkers  finished  markers %d    markersIdSet %d ",markers_.size(),markersIdSet.size());
}

/**
 * @name: checkMarkersId
 * @brief: 检查二维码的ID是否在地图中
 *         原理：通过ID号，反向找到地图配置文件中对应的二维码信息，
 *         计算同一批次的二维码的第一个点之间的距离，如果该距离大于单片二维码
 *         的空间最大距离，那就说明这个定位一定是有问题的
 * @Date: 2021-03-02 10:09:38
 * @params {*}
 * @return {*}
 */
/**
 * @name: 
 * @brief: 
 * @Date: 2021-03-25 10:17:26
 * @param {const} std
 * @return {*}
 */

void ArucoSingleMarker::getCameraPose(aruco::Marker& marker)
{
    cv::Mat poseRtMat;
    // vision_msgs::ArucoMarkersMapPose arucoMarkersMapPoseMsg;
    // ROS_INFO("0000000000");
    bool  re= tracker_.estimatePose(marker  , cameraParameters_, markerSize_);
     // ROS_INFO("11111111111");
    if(re)
    {
        //  ROS_INFO("222222222");
            poseRtMat = tracker_.getRTMatrix().inv();
          //    ROS_INFO("3333333333333");
            // 这里取逆有历史遗留问题
            //std::cout<<"poseRtMat:"<<poseRtMat<<std::endl;
           // poseRtMat =  coordinatesPoseMatMap_[targetCoordinate_] * poseRtMat;
             // ROS_INFO("44444444444444");
            //std::cout<<"poseRtMat:"<<poseRtMat<<std::endl;
            
            
            convertRtMatToMarkersPoseMsg(poseRtMat);
                //   ROS_INFO("555555555555");
     }
    else
    {
        ROS_WARN("single mark  estimate pose  failure");
    }
}


/**
 * @name: 
 * @brief: 将RT矩阵转换为Pose Msg
 * @Date: 2021-03-05 10:36:05
 * @params {*}
 * @return {*}
 */

void ArucoSingleMarker::convertRtMatToMarkersPoseMsg(const cv::Mat& poseRtMat)                                         
{
    Eigen::Matrix3f rotMatrix;
    geometry_msgs::PoseStamped cameraPose3D;
    vision_msgs::ArucoMarkersMapPose arucoMarkersMapPoseMsg;

    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            rotMatrix(i,j)=poseRtMat.at<float>(i,j);

    cameraPose3D.header.stamp = captureTimeStamp_;
    cameraPose3D.pose.position.x = poseRtMat.at<float>(0, 3);
    cameraPose3D.pose.position.y = poseRtMat.at<float>(1, 3);
    cameraPose3D.pose.position.z = poseRtMat.at<float>(2, 3);

  

    Eigen::Quaternionf q(rotMatrix);
    cameraPose3D.pose.orientation.x = q.x();
    cameraPose3D.pose.orientation.y = q.y(); 
    cameraPose3D.pose.orientation.z = q.z();
    cameraPose3D.pose.orientation.w = q.w();

    arucoMarkersMapPoseMsg.header.stamp = captureTimeStamp_;
    arucoMarkersMapPoseMsg.quantityOfMarkers = numbersOfMarkers_;
    arucoMarkersMapPoseMsg.pose = cameraPose3D;

   // cout<<"arucoMarkersMapPoseMsg"<<arucoMarkersMapPoseMsg.pose<<endl;
    arucoMarkersMapPosePub_.publish(arucoMarkersMapPoseMsg); 

    //  以消息的形式  发布  pose
    //  同时以tf的形式 发布相机和二维码之间的tf
     tf::Quaternion q_tf(q.x( ), q.y( ),  q.z( ) ,  q.w( ));

     stamp_tranfrom.child_frame_id_=child_frame_id;
     stamp_tranfrom.frame_id_=frame_id;
     stamp_tranfrom.stamp_=captureTimeStamp_;
     stamp_tranfrom.setOrigin( tf::Vector3( cameraPose3D.pose.position.x,  cameraPose3D.pose.position.y,  cameraPose3D.pose.position.z));
     stamp_tranfrom.setRotation(q_tf);

     tf_broadcaster.sendTransform(stamp_tranfrom);  
     
     /*
     tf_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
        ros::Time::now(),"base_link", "base_laser"));
       stamp_tranfrom(transform,   captureTimeStamp_, )*/
     


   // tranfrom.child_frame_id_=child_frame_id;
   // tranfrom.frame_id_=frame_id;
     //stamp_tranfrom.

}

/**
 * @name: 
 * @brief: 
 * @Date: 2021-03-16 16:41:03
 * @param {*}
 * @return {*}    二维码坐标系变换矩阵计算
 */
void ArucoSingleMarker::getMatFromPose(cv::Mat& m44,
                                  const Pose& pose)
{
    Eigen::Vector3d eulerAngles{pose.yaw, 
                                pose.pitch,
                                pose.roll};
    Eigen::Matrix3d  rotationMatrix;
    Euler2Rotation(eulerAngles, rotationMatrix);

    m44 = cv::Mat::eye(4, 4, CV_32FC1);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m44.at<float>(i, j) = rotationMatrix(i,j);

    // now, add translation information
    m44.at<float>(0, 3) = pose.x;
    m44.at<float>(1, 3) = pose.y;
    m44.at<float>(2, 3) = pose.z;
}

/**
 * @name: Euler2Rotation
 * @brief: 欧拉角转换为旋转矩阵
 * @params {*}
 * @return {*}
 */
void ArucoSingleMarker::Euler2Rotation(const Eigen::Vector3d& eulerAngles, 
                                   Eigen::Matrix3d& rotationMatrix)
{
    rotationMatrix =   Eigen::AngleAxisd(eulerAngles[0], Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd(eulerAngles[1], Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(eulerAngles[2], Eigen::Vector3d::UnitX());
}



}