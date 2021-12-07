//该类  实现单个二维码姿态检测   并根据激光雷达对车体的定位  计算 单个二维码在世界坐标系下的位置
#ifndef  ARUCO_SINGLE_MARKER_HPP__
#define ARUCO_SINGLE_MARKER_HPP__
#include <iostream>
#include <aruco.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include  <vision_msgs/Pose2DStamped.h>
#include <vision_msgs/CameraControl.h>
#include <vision_msgs/ArucoMarkersMapPose.h>
#include <geometry_msgs/Pose2D.h>
#include  <string.h>

#include <math.h>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <chrono>   
#include <yaml-cpp/yaml.h>

using namespace std;
namespace  aruco_markers
{
   //单个二维码检测的参数接口
    struct  ArucoSingleMarkerParam
        {   
                std::string  topic_cameral_control;   //订阅相机控制topic名称
                std::string  topic_markers_map_pose;  //二维码姿态topic名称
                std::string   camera_topic_name;  //图像话题名称
                std::string    camera_param_file_path;    //相机参数文件路径
                std::string   camera_param_file_name;    //相机参数文件名
                std::string   camera_detect_topic_name; // 二维码检测结果 topic名称
                std::string   map_file_path;    //二维码地图文件路径
                std::string   map_file_name;  //二维码地图文件名称
                std::string  markers_classify_file_path;   //二维码分类文件路径
                std::string  markers_classify_file_name;  //二维码分类文件名称
                bool   localizationEnable;
                int cameraIndex_; // 相机右：0 左：1 后：2      
                bool debug;
                float markerSize;   //二维码边长
                cv::Rect   rect_roi;     //通过设置有效区域裁出单个二维码
                std::string child_frame_id;
                std::string frame_id;
        };

     class  ArucoSingleMarker
     {
          struct Pose
    {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
    };   //不同二维码坐标系  之间的姿态

        public:   
               ArucoSingleMarker(ros::NodeHandle& nh,  ArucoSingleMarkerParam&  param);
               ArucoSingleMarker() = default;
              aruco_markers::ArucoSingleMarkerParam    aruco_single_marke_param;
        private:
            void imageCallback(const sensor_msgs::ImageConstPtr& msg);
            void localizationEnableCallback(const vision_msgs::CameraControl &msg);
            void detectMarkers(const cv::Mat& inImage, std::set<std::size_t>& markersIdSet);
            void checkMarkersId(const std::vector<aruco::Marker>& markers);
            void getCameraPose(aruco::Marker&  marker);

            // void classfiyIdWall(const std::set<std::size_t>& markersId);
            void convertRtMatToMarkersPoseMsg(const cv::Mat& poseRtMat);
            void getMatFromPose(cv::Mat& m44,const Pose& pose);
                                        
            void Euler2Rotation(const Eigen::Vector3d& eulerAngles, 
                                Eigen::Matrix3d& rotationMatrix);
                                                 
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle nhPriv_;

            image_transport::ImageTransport it_;
            image_transport::Subscriber imageSub_;     //图像订阅器
            image_transport::Publisher imagePub_;
            
            ros::Subscriber cameraContolSub_;
            ros::Publisher    cameraPose2DPub_; 
            ros::Publisher    cameraPose3DPub_; 
            ros::Publisher    arucoMarkersMapPosePub_; 

            std::string cameraCalibrationFilePath_;
            std::string markerMapFilePath_;     
            int cameraIndex_; // 相机右：0 左：1 后：2
            bool localizationEnable_{false};
            
            aruco::CameraParameters  cameraParameters_; //
            aruco::MarkerDetector   markerDetector_;   //aruco库类  负责图像二维码 id识别和角点检测
            aruco::MarkerPoseTracker    tracker_;
            std::vector<aruco::Marker> markers_;    //输入一帧图像  检测到的二维码 2d信息保存在  markers_
            float markerSize_;
            std::size_t  numbersOfMarkers_;      //经过过滤后  检测到的二维码数目
            //unsigned   相当于无符号整型   例如:bitset的size()操作返回bitset对象中二进制位中的个数，返回值类型是size_t。
            cv::Mat inImage_;
            // cv::Mat inRGBImage_;
            vision_msgs::ArucoMarkersMapPose arucoMarkersMapPoseMsg_;
            ros::Time  captureTimeStamp_;
            std::map<int, aruco::Marker3DInfo> mapInfo_;
            std::set<std::size_t> currentMarkersIdSet_; //关联容器   set(关键字即值，即只保存关键字的容器)  不同于map(关联数组：保存关键字----值对)
            // 对应的每个坐标系的二维码个数
            std::vector<std::size_t> coordinatesMarkerQuantity_;
            // 所有不同坐标系下的二维码ID set
            std::map<std::string, std::size_t> coordinatesMarkerQuantityMap_;           //坐标系及对应的二维码数目
            std::map<std::string, std::set<std::size_t>> coordinatesMarkerSetMap_;      //坐标系  及对于的 二维码id集合
            std::map<std::string, cv::Mat> coordinatesPoseMatMap_;    //坐标系 及对于的  姿态矩阵 Mat 4×4  
            std::string targetCoordinate_;     //当前检测到的二维码属于 坐标系名称    镇江坐标系命名样列： coordinate1    coordinate2  coordinate3
            std::bitset<32> coordinatesBitSet_;
    
            static constexpr double PI_{3.1415926};
            bool debug_;

            //二维码和相机  tf发布
            tf::TransformBroadcaster   tf_broadcaster;
            tf::StampedTransform  stamp_tranfrom;  
            std::string  frame_id;     //tf 父坐标frame
            std::string  child_frame_id;     //tf子坐标frame

     };


}


#endif