
#include "aruco_markers/aruco_markers.hpp"

using namespace cv;
using namespace aruco;
using namespace std;
using namespace chrono;

namespace aruco_markers
{
   bool   ArcucoMarkersInterFace::import_config(string&   configFilePath_)
   {
            CamNum=0;
            YAML::Node  configNode_ = YAML::LoadFile(configFilePath_);   //  YAML::Node
            if(configNode_.IsNull())
            {
                    ROS_INFO("ArucoMarkers  load YAML  File  error!!!!");
                    return false ;
            } 
            YAML::iterator  it;
            it=configNode_["aruco_markers_detect_config"].begin();
            CamNum=it->second["camera_num"].as<int>();
            if(CamNum<=0)
            {
                    ROS_INFO("CAMERA_NUM  set  0  or   CAMERA_NUM  load  error !!!!");
                    return false ;
            } 
            
             std::string  camera_param_file_path =  it->second["camera_calibration_file_path"].as<std::string>();
             std::string    map_file_path=it->second["map_file_path"].as<std::string>();
             std::string   map_file_name=it->second["map_file_name"].as<std::string>();
           std::string markers_classify_file_path =it->second["markers_classify_file_path"].as<std::string>();
           std::string markers_classify_file_name =it->second["markers_classify_file_name"].as<std::string>();
           std::string topic_cameral_control  =it->second["topic_cameral_control"].as<std::string>();
           
           bool debug_=it->second["debug"].as<bool>();

           float markerMapSliceSize=it->second["markerMapSliceSize"].as<float>();

            std::vector<std::string>  camera_param_file_names;       
            std::vector<std::string>  camera_topic_names;
            std::vector<std::string>   markers_map_pose_topic;
            std::vector< std::string> camera_detect_topic_names;
            std::vector<bool>  localizationEnables_; 
             
            for(int cam_id=0; cam_id<CamNum;  cam_id++)
            {
                    std::string f_n=it->second["camera_calibration_file_names"][cam_id].as<std::string>();         
                    if(f_n.empty())
                       {
                                ROS_INFO("camera_calibration_file_name  %d   is  empty      load  error !!!!",cam_id);
                                return false ;
                       }
                    else
                        {
                                camera_param_file_names.push_back(f_n);

                        }
                    std::string  topic_name=it->second["camera_topic_name"][cam_id].as<std::string>();
                    if(topic_name.empty())
                       {
                                ROS_INFO("camera_topic_name  %d   is  empty      load  error !!!!",cam_id);
                                return false ;
                       }
                    else
                      {
                             camera_topic_names.push_back(topic_name);
                            ROS_INFO_STREAM("topic_name:"<<topic_name);
                      }
                    std::string  mark_pose_name=it->second["markers_map_pose_topic"][cam_id].as<std::string>();
                    if(mark_pose_name.empty())
                       {
                                ROS_INFO("mark_pose_name  %d   is  empty      load  error !!!!",cam_id);
                                return false ;
                       }
                    else
                      {
                             markers_map_pose_topic.push_back(mark_pose_name);
                            ROS_INFO_STREAM("mark_pose_name:"<<mark_pose_name);
                      }

                    std::string  camera_detect_topic_name=it->second["camera_detect_topic_name"][cam_id].as<std::string>();
                    if(camera_detect_topic_name.empty())
                       {
                                ROS_INFO("camera_detect_topic_name  %d   is  empty      load  error !!!!",cam_id);
                                return false ;
                       }
                    else
                      {
                             camera_detect_topic_names.push_back(camera_detect_topic_name);
                            ROS_INFO_STREAM("camera_detect_topic_name:"<<camera_detect_topic_name);
                      }

                    bool  localizationEnable_=it->second["localization_enable"][cam_id].as<bool>();
                
                    localizationEnables_.push_back(localizationEnable_);
                     ROS_INFO("camera_id: %d  ,localizationEnables_:%d. ",cam_id,localizationEnables_[cam_id]);
                    std::cout<<"camera_id:"<<cam_id<<" localizationEnables_: "<<localizationEnables_[cam_id]<<std::endl;        
            }

         for(int cam_id=0; cam_id<CamNum;  cam_id++){
                Aruco_Marker_Param  param_temp;
                param_temp.camera_param_file_name  =  camera_param_file_names[cam_id];
                param_temp.camera_param_file_path    =  camera_param_file_path;
                param_temp.map_file_path  =  map_file_path;
                param_temp.map_file_name = map_file_name;
                param_temp.markers_classify_file_path  = markers_classify_file_path;
                param_temp.markers_classify_file_name =  markers_classify_file_name;
                param_temp.topic_cameral_control  =   topic_cameral_control;
                param_temp.topic_markers_map_pose = markers_map_pose_topic[cam_id];
                param_temp.debug = debug_;
                param_temp.cameraIndex_  =  cam_id;
                param_temp.localizationEnable = localizationEnables_[cam_id];
                param_temp.camera_detect_topic_name  =  camera_detect_topic_names[cam_id];
                param_temp.markerMapSliceSize =markerMapSliceSize;
                param_temp.camera_topic_name= camera_topic_names[cam_id];
                aruco_marker_params.push_back(param_temp);
         }

        ROS_INFO_STREAM("ArcucoMarkersInterFace  import_config  is finish. ");
        return  true;
}
    
    ArcucoMarkersInterFace:: ArcucoMarkersInterFace(ros::NodeHandle& nh)
    {
      std::string  file_path ="/data/vision_ws/config/aruco_markers_detect_config.yaml";
      import_config(file_path);
      for(int cam_id=0;cam_id<CamNum;cam_id++)
      {
        boost::shared_ptr<ArucoMarkers>   aruco_mark_ptr  =     boost::make_shared<ArucoMarkers>(nh,aruco_marker_params[cam_id]);
        aruco_markers_ptr.push_back(aruco_mark_ptr);
      } 

      ros::Rate loop_rate(10);
        while (ros::ok())
        {
            ros::spinOnce();     
            loop_rate.sleep();
        }
    }

    ArcucoMarkersInterFace::~ArcucoMarkersInterFace(){
        ;
    }
}


