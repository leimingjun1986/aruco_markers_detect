/*
 * @Descripttion: 
 * @version: 
 * @Author: leimingjun
 * @Date: 2021-11-09 
 * @LastEditors: pifan
 * @LastEditTime: 2021-03-24 15:44:32
 */

#include "aruco_marker_map_build/aruco_single_mark.hpp"

using namespace aruco_markers;

bool   import_config(string&   configFilePath_ ,  vector<ArucoSingleMarkerParam>&  params)
   {
            ROS_INFO("improt config 0000000000");
            int  CamNum=0;
            YAML::Node  configNode_ = YAML::LoadFile(configFilePath_);   //  YAML::Node
            if(configNode_.IsNull())
            {
                    ROS_INFO("ArucoSingleMarker load YAML  File  error!!!!");
                    return false ;
            } 
            YAML::iterator  it;
            it=configNode_["aruco_single_detect_config"].begin();
            CamNum=it->second["camera_num"].as<int>();
            if(CamNum<=0)
            {
                    ROS_INFO("CAMERA_NUM  set  0  or   CAMERA_NUM  load  error !!!!");
                    return false ;
            } 
            
             ROS_INFO("1111111111111111");
           std::string  camera_param_file_path =  it->second["camera_calibration_file_path"].as<std::string>();
           std::string topic_cameral_control  =it->second["topic_cameral_control"].as<std::string>();
           
           bool debug_=it->second["debug"].as<bool>();

           float markerSize=it->second["markerSize"].as<float>();

            std::vector<std::string>  camera_param_file_names;       
            std::vector<std::string>  camera_topic_names;
            std::vector<std::string>   markers_map_pose_topic;
            std::vector< std::string> camera_detect_topic_names;
            std::vector<bool>  localizationEnables_; 
            std::vector<pair<std::string,std::string>>  tfcamtomark;
            
              ROS_INFO("222222222222222222");
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
                    
                    std::pair<std::string,std::string>  tfcam2mark;
                    tfcam2mark.first =it->second["tfFrameToChildFrame"][2*cam_id].as<string>();
                    tfcam2mark.second=it->second["tfFrameToChildFrame"][2*cam_id+1].as<string>(); 
                    tfcamtomark.push_back(tfcam2mark);

                   ROS_INFO_STREAM("tf  frame_id:"<<tfcam2mark.first );
                   ROS_INFO_STREAM("tf  child_frame_id:"<<tfcam2mark.second );
            }
              ROS_INFO("3333333333333333333333333");
         for(int cam_id=0; cam_id<CamNum;  cam_id++){
                ArucoSingleMarkerParam  param_temp;
                param_temp.camera_param_file_name  =  camera_param_file_names[cam_id];
                param_temp.camera_param_file_path    =  camera_param_file_path;
                param_temp.topic_cameral_control  =   topic_cameral_control;
                param_temp.topic_markers_map_pose = markers_map_pose_topic[cam_id];
                param_temp.debug = debug_;
                param_temp.cameraIndex_  =  cam_id;
                param_temp.localizationEnable = localizationEnables_[cam_id];
                param_temp.camera_detect_topic_name  =  camera_detect_topic_names[cam_id];
                param_temp.markerSize=markerSize;
                param_temp.camera_topic_name= camera_topic_names[cam_id];
                param_temp.frame_id  = tfcamtomark[cam_id].first;
                param_temp.child_frame_id  = tfcamtomark[cam_id].second;
                params.push_back(param_temp);
         }

        ROS_INFO_STREAM("aruco single marker  import_config  is finish. ");
        return  true;
}
  

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_markers");
    ros::NodeHandle nh;
    vector<ArucoSingleMarkerParam>  params;
    std::string  file_path ="/home/flfjepl/catkin_ws_vision_location/src/aruco_markers_detect/config/aruco_single_detect_config.yaml";
    import_config(file_path,params);
    std::shared_ptr<ArucoSingleMarker>  arucoMarkerPtr_ = std::make_shared<ArucoSingleMarker>(nh, params[0]);
     ros::Rate loop_rate(10);
     while (ros::ok())
    {
        ros::spinOnce();     
        loop_rate.sleep();
    }
    return 0;
}