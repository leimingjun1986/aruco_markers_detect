#include  <nodelet/nodelet.h>
#include  <aruco_markers/aruco_markers.hpp>
 
 namespace  aruco_markers{

        // nodelet  接口类
        class  aruco_markers_nodelet:public nodelet::Nodelet      
        {
            public:
            aruco_markers_nodelet();
            ~ aruco_markers_nodelet();
            
            boost::shared_ptr<ArcucoMarkersInterFace>   arcuco_marker_interface_ptr;

            private:
            virtual void onInit();    //此函数声明部分为固定格式，在nodelet加载此plugin会自动执行此函数

        };

 }