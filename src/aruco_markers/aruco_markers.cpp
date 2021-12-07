/*
 * @Descripttion: 
 * 镇江组间车间项目：
 * 将所有的二维码地图分为三个部分，这么做的目的是将大地图进行拆分，
 * 防止定位算法因为之前地图过大出现的异常,以room_wall的局部坐标系为二维码全局坐标系
 * 
 * --------------------------
 *       coordinate1         |
 *                           |
 *                           |
 *                           | coordinate2
 *                           | 
 *                           | 
 *                           |
 *                           |--------------
 *                                         | 
 *                          coordinate3
 *                             (全局)      | 
 *                                         |
 *                            --------------
 * @version: 
 * @Author: pifan
 * @Date: 2021-03-01 19:02:13
 * @LastEditors: pifan
 * @LastEditTime: 2021-03-16 13:51:30
 */
#include  <aruco_markers/aruco_markers.hpp>

using namespace cv;
using namespace aruco;
using namespace std;
using namespace chrono;
namespace aruco_markers
{
    //ArucoMarkers类构造函数
ArucoMarkers::ArucoMarkers(ros::NodeHandle& nh, Aruco_Marker_Param&  param):nh_(nh),
                                                nhPriv_("~"),
                                                it_(nhPriv_) 
{
    aruco_marker_param =  param;   //参数结构体传入
    cameraIndex_ =aruco_marker_param.cameraIndex_;
    ROS_INFO("cameraIndex_: %d",cameraIndex_);
    std::string   image_topic = aruco_marker_param.camera_topic_name;
    ROS_INFO("image_topic: %s",image_topic.c_str());
    imageSub_ = it_.subscribe( image_topic,  1,  &ArucoMarkers::imageCallback,  this);    //订阅图像
    ROS_INFO("camera_detect_topic_name: %s",aruco_marker_param.camera_detect_topic_name.c_str());
    imagePub_ = it_.advertise(aruco_marker_param.camera_detect_topic_name,  1);      //  检测结果图像 topic  
    ROS_INFO("topic_cameral_control: %s",aruco_marker_param.topic_cameral_control.c_str());
    cameraContolSub_ = nhPriv_.subscribe(aruco_marker_param.topic_cameral_control, 1, &ArucoMarkers::localizationEnableCallback, this);  //订阅相机控制信号
    ROS_INFO("topic_markers_map_pose: %s",aruco_marker_param.topic_markers_map_pose.c_str());
    arucoMarkersMapPosePub_ = nhPriv_.advertise<vision_msgs::ArucoMarkersMapPose>(aruco_marker_param.topic_markers_map_pose, 1);   //多个topic  名字不同
   
    cameraCalibrationFilePath_ = aruco_marker_param.camera_param_file_path +  aruco_marker_param.camera_param_file_name;
     ROS_INFO("cameraCalibrationFilePath_: %s",cameraCalibrationFilePath_.c_str());
    
   markerMapFilePath_ = aruco_marker_param.map_file_path+aruco_marker_param.map_file_name;
   ROS_INFO("markerMapFilePath_: %s",markerMapFilePath_.c_str());
    
   localizationEnable_   =  aruco_marker_param.localizationEnable ;
   //二维码分类配置文件路径
   markerClassifyFilePath_=aruco_marker_param.markers_classify_file_path + aruco_marker_param.markers_classify_file_name;
    ROS_INFO("markerClassifyFilePath_: %s",markerClassifyFilePath_.c_str());
    debug_= aruco_marker_param.debug;

    cameraParameters_.readFromXMLFile(cameraCalibrationFilePath_);
    markerMapConfig_.readFromFile(markerMapFilePath_);

    //只需要传递字典名称
    markerDetector_.setDictionary(markerMapConfig_.getDictionary());

    if (cameraParameters_.isValid() && markerMapConfig_.isExpressedInMeters())
    {
        tracker_.setParams(cameraParameters_, markerMapConfig_);
        markerSize_ = cv::norm(markerMapConfig_[0][0]- markerMapConfig_[0][1]);
    }
    mapInfo_ = tracker_.getMapInfo();

    // 将二维码配置文件加载进来
    loadConfig(YAML::LoadFile(markerClassifyFilePath_));
    markerMapSliceSize_ =aruco_marker_param.markerMapSliceSize;

    ROS_INFO("Camera id %d : ArucoMarkers  construct  finish",aruco_marker_param.cameraIndex_);
}  

/**
 * @name: 
 * @brief: 
 * @Date: 2021-03-23 20:48:30
 * @param {const} YAML
 * @return {*}
 */
bool ArucoMarkers::loadConfig(const YAML::Node& yamlNode)
{
    for (auto it = yamlNode["coordinates_markers_quantity"].begin(); it!=yamlNode["coordinates_markers_quantity"].end(); ++it)
    {
        coordinatesMarkerQuantityMap_[it->first.as<string>()] = it->second.as<std::size_t>();
    }

    // size_t cnt = 0;
    for (auto it = yamlNode["coordinates_markers_set"].begin(); it!=yamlNode["coordinates_markers_set"].end(); ++it)
    {
        auto iter = coordinatesMarkerQuantityMap_.find(it->first.as<string>());
        size_t quantity;
        if (iter != coordinatesMarkerQuantityMap_.end())  
            quantity= iter->second;
        else 
            throw cv::Exception(9904,  "YAML FILES ERROR", 
                                "ArucoMarkers::loadConfig",  __FILE__, __LINE__);
            
        set<size_t> markerSet;
        for (size_t i=0; i<quantity; ++i)
        {
            markerSet.insert(it->second[i].as<std::size_t>());
        }
        coordinatesMarkerSetMap_[it->first.as<string>()] = markerSet;

    }

    for (auto it = yamlNode["coordinates_TF"].begin(); it!=yamlNode["coordinates_TF"].end(); ++it)
    {
        Pose pose;
        pose.x = it->second[0].as<double>();
        pose.y = it->second[1].as<double>();
        pose.z = it->second[2].as<double>();
        pose.yaw = it->second[3].as<double>();
        pose.pitch = it->second[4].as<double>();
        pose.roll = it->second[5].as<double>();
        cv::Mat poseMat;
        getMatFromPose(poseMat, pose);
        coordinatesPoseMatMap_[it->first.as<string>()] = poseMat;
    }
        
    return true;

}
/**
 * @name: imageCallback
 * @brief: 图像回调函数
 * @Date: 2021-03-02 09:57:48
 * @params {*}
 * @return {*}
 */
void ArucoMarkers::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (localizationEnable_ == false)            //定位开关  通过launch文件配置     他通过/camera_control 消息  控制定位开关的开关
       {
           //ROS_INFO("camera_id  %d  imageCallback",aruco_marker_param.cameraIndex_);
           return;
       }
        
    try
    {
       // std::cout<<"camera  id:"<<cameraIndex_<<"  marker detect."<<std::endl;
        auto start = system_clock::now();
        captureTimeStamp_ = msg->header.stamp;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,  sensor_msgs::image_encodings::MONO8);//RGB8;                                              
        inImage_ = cv_ptr->image;
        // 检测二维码，并且当前定位到的marker的ID提取出来
        detectMarkers(inImage_, currentMarkersIdSet_); //输入 inImage_,   输出  currentMarkersIdSet_ 二维码id集合
        // 检查二维码ID是否在正常范围内，防止出现误判   只给出了打印信息
        checkMarkersId(markers_);    //markers_  检测到的所有二维码   在上一个函数中计算得到         没有经过与二维码库id比对过滤    
        // 对获取的二维码进行分类
        classfiyMarkersCoordinates(currentMarkersIdSet_);   //currentMarkersIdSet_   经过过滤后的二维码id集合
        // classfiyIdWall(currentMarkersIdSet_);
        // 获取相机的在全局二维码坐标系下的位姿    重要计算函数
        getCameraPose(markers_);    //传入未过滤的二维码id及角点？

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
void ArucoMarkers::localizationEnableCallback(const vision_msgs::CameraControl &msg)
{
    uint8_t controlByte = msg.controlByte;
     (controlByte>>cameraIndex_) & 0x01  ? localizationEnable_ = true : localizationEnable_ = false;  
    std::cout<<cameraIndex_<<"    enter  cameracontrol callback"<<"  localizationEnable_:"<<localizationEnable_<<std::endl;
}



bool ArucoMarkers::markersIdSetIntersection(const std::set<std::size_t>& setL,
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
    
}

 /**
  * @name: 
  * @brief: 区分二维码的坐标系
  * @Date: 2021-03-25 09:40:55
  * @param {const} std
  * markersIdSet： 输入的当前定位得到的二维码
  * @return {*}
  */
 void ArucoMarkers::classfiyMarkersCoordinates(const std::set<std::size_t>& markersIdSet)
 {
    coordinatesBitSet_.reset();
    size_t pos = 0;
    for (auto it = coordinatesMarkerSetMap_.begin(); it!=coordinatesMarkerSetMap_.end(); ++it)
    {
        bool isInter = markersIdSetIntersection(markersIdSet, it->second);
        if (isInter)
        {
            coordinatesBitSet_.set(pos);
            targetCoordinate_ = it->first;
        }
        pos++;
    }

 }                                 





/**
 * @name: 
 * @brief: 检测二维码
 * @Date: 2021-03-04 11:36:06
 * @params {*}
 * @return {*}
 * @param {const} cv
 */
void ArucoMarkers::detectMarkers(const cv::Mat& inImage,
                                 std::set<std::size_t>& markersIdSet)    //输入图像   输出二维码id   经过了过滤
{
    markersIdSet.clear();
    // 检测二维码
    markers_ = markerDetector_.detect(inImage);    // std::vector<aruco::Marker> markers_   二维码的图像二维信息 
    // 在估计姿态的时候还会过滤一次   返回  vector<int>类型    检测到的二维码与 预先输入的 MarkerMap二维码 id做比较 
    auto markersId = markerMapConfig_.getIndices(markers_);    //MarkerMap : public std::vector<Marker3DInfo>    markerMapConfig_
    std::set<std::size_t> markersIndexSet = {markersId.begin(), markersId.end()};
    numbersOfMarkers_ = markersIndexSet.size();
    
    // 如果检查ID发现是特殊的ID，就将isSpecialId_设置为true
    // checkSpecialId(markersIndexSet);
    cv::Mat inRGBImage;
    // 将灰度图转换为RGB图
    cv::cvtColor(inImage, inRGBImage, COLOR_GRAY2RGB);    //占用了计算时间
    // 将识别的二维码结果在图像上做标记
    // 打印二维码ID，并对图像做标记
    for (std::size_t idx : markersIndexSet)
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
void ArucoMarkers:: checkMarkersId(const std::vector<aruco::Marker>& markers)
{
    std::vector<cv::Point3f> p3d;
    for (const auto& marker : markers)
    {
        // 如果检测到二维码id在地图中
        if (mapInfo_.find(marker.id) != mapInfo_.end())   // std::map<int, aruco::Marker3DInfo> mapInfo_;
        {
            // 将检测到二维码的3d信息中对应的第一个点存起来
            p3d.push_back(mapInfo_[marker.id].points[0]);
        }       
    }

    if (p3d.size())
    {
        auto firstPoint = p3d[0];
        for ( auto p : p3d)
        {
            float distance = cv::norm(firstPoint - p);
            if (distance > markerSize_*markerMapSliceSize_) 
                throw cv::Exception(9901,  "Unexpected marker detected", 
                                    "ArucoMarkers:: checkMarkersId",  __FILE__, __LINE__);                            
        }

    }
}


/**
 * @name: 
 * @brief: 
 * @Date: 2021-03-25 10:17:26
 * @param {const} std
 * @return {*}
 */
void ArucoMarkers::getCameraPose(const std::vector<aruco::Marker>& markers)
{
    cv::Mat poseRtMat;
    // vision_msgs::ArucoMarkersMapPose arucoMarkersMapPoseMsg;
    if (tracker_.isValid())
    {
        if (tracker_.estimatePose(markers))
        {
            poseRtMat = tracker_.getRTMatrix().inv();
           // cout << "coordinatesBitSet_.count(): " <<coordinatesBitSet_.count() << endl;
            if (coordinatesBitSet_.count() == 1)
            {
                // 这里取逆有历史遗留问题
               //std::cout<<"poseRtMat:"<<poseRtMat<<std::endl;
                poseRtMat =  coordinatesPoseMatMap_[targetCoordinate_] * poseRtMat;
                //std::cout<<"poseRtMat:"<<poseRtMat<<std::endl;
                convertRtMatToMarkersPoseMsg(poseRtMat);
            }
            else
            {
                throw cv::Exception(9902,  "DUPLICATE COODIBATES", 
                                    "ArucoMarkers::getCameraPose",  __FILE__, __LINE__);
            }
        }
    }

}


/**
 * @name: 
 * @brief: 将RT矩阵转换为Pose Msg
 * @Date: 2021-03-05 10:36:05
 * @params {*}
 * @return {*}
 */
void ArucoMarkers::convertRtMatToMarkersPoseMsg(const cv::Mat& poseRtMat)                                         
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
}


/**
 * @name: 
 * @brief: 
 * @Date: 2021-03-16 16:41:03
 * @param {*}
 * @return {*}    二维码坐标系变换矩阵计算
 */
void ArucoMarkers::getMatFromPose(cv::Mat& m44,
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
void ArucoMarkers::Euler2Rotation(const Eigen::Vector3d& eulerAngles, 
                                   Eigen::Matrix3d& rotationMatrix)
{
    rotationMatrix =   Eigen::AngleAxisd(eulerAngles[0], Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd(eulerAngles[1], Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(eulerAngles[2], Eigen::Vector3d::UnitX());
}



}