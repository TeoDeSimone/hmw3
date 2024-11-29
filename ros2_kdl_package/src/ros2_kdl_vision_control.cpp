#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>


#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
#include "aruco/markerdetector.h"
#include "aruco/aruco.h"
#include "aruco/cameraparameters.h"
											
#include "sensor_msgs/msg/image.hpp"        //\\    //
#include "std_msgs/msg/header.hpp"		   //  \\  //
#include <opencv2/features2d.hpp>	
#include <cv_bridge/cv_bridge.h> 				//\\ 		cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> //  \\  //	Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> 				  //    \\// 	We include everything about OpenCV as we don't care much about compilation time at the moment.
#include <opencv2/aruco.hpp>


 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;
using std::placeholders::_1;

class iiwa_vision_task : public rclcpp::Node
{

	public:
		iiwa_vision_task() : Node("ros2_kdl_vision_control") 
		{

			std::cout<<"test:constructor\n";
			
			
			subscriber_img_=
        		this->create_subscription<sensor_msgs::msg::Image>(
          		"/camera", 10, std::bind(&iiwa_vision_task::subscriber_img, this, _1));
          	/*
          	subscriber_camera_info_=
        		this->create_subscription<sensor_msgs::msg::CameraInfo>(
          		"/camera_info", 10, std::bind(&iiwa_vision_task::subscriber_camera_info, this, _1));
          	*/

          	timer_ = this->create_wall_timer(
		        500ms, std::bind(&iiwa_vision_task::timer_callback, this));
		
			
		}



	private:
		
		void subscriber_img(const sensor_msgs::msg::Image & camera_msg) //Secondo me qua dentro va messo msg_, se vedete l'api mi pare ci sia un puntatore, verificate
	    {
	      cv_ptr_ = cv_bridge::toCvCopy(camera_msg,"bgr8");
	      cv_img_ = *cv_ptr_;

	    }


/*	    void subscriber_camera_info(const sensor_msgs::msg::CameraInfo & camera_info) //Secondo me qua dentro va messo msg_, se vedete l'api mi pare ci sia un puntatore, verificate
	    {
	    	if(!cam_info_received)
	    			
	     	

	    	cam_info_received = true;
	    }
	    
*/
		void timer_callback() 
		{

		
			std::cout<<"test:timer_callback\n";

			//cv::imshow("cameraPOV", cv_img_.image);
			//cv::waitKey(0);

			cv::Mat image_marked;

			image_marked = cv_img_.image; 

			markers_ = mDetector_.detect(cv_img_.image);
			std::cout<<"markerID:"<<markers_[0].id<<'\n';

			markers_[0].draw(image_marked, cv::Scalar(0, 0, 255), 2);


			/*
			cv::imshow("cameraPOV", image_marked);
			cv::waitKey(0);
*/
			cameraPose_ = poseTracker.estimatePose(markers_);


			std::cout<<"cmaera pose:"<<cameraPose_; 


		}


		std::vector<aruco::Marker> markers_;
		aruco::MarkerDetector mDetector_;
		aruco::CameraParameters cam_params_;


		bool cameraPose_;

		aruco::MarkerMapPoseTracker poseTracker;

		//

		cv_bridge::CvImagePtr cv_ptr_;
		cv_bridge::CvImage cv_img_;
		
		/*std::vector<int> markerIds_;
		std::vector<std::vector<cv::Point2f>> markerCorners_, rejectedCandidates_;
		cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
		*/



		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_img_;
		rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscriber_camera_info_;
		bool cam_info_received;


};

int main(int argc, char *argv[]) 
{
	std::cout<<"test:main\n";
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<iiwa_vision_task>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}