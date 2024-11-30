#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Dense>
#include <iostream>
#include "aruco/markerdetector.h"
#include "aruco/aruco.h"
#include "aruco/posetracker.h"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"




class VisionController : public rclcpp::Node {
public:
    VisionController() : Node("ros2_kdl_vision_control") 
    {




        iteration_ = 0;//this will take account of number of cmd_publisher iterations
        t_ = 0;
        joint_state_available_ = false; 
        



        // Publisher to send velocity commands to the robot
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/velocity_controller/commands", 10);
        
        // Subscriber to get camera frames
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", 10, std::bind(&VisionController::cameraInfoCallback, this, std::placeholders::_1));
        
        // Subscriber to get camera frames
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera", 10, std::bind(&VisionController::imageCallback, this, std::placeholders::_1));
            

        // Load ArUco dictionary
        aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        detector_params_ = cv::aruco::DetectorParameters::create();

        RCLCPP_INFO(this->get_logger(), "Vision Controller Node Started");
    }

private:
    
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        
      if(camera_state_available_){

        // Convert ROS image to OpenCV format
        try {
            input_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Detect ArUco markers
        std::vector<aruco::Marker> det;
        
        //aruco::CameraParameters cam_params(camera_matrix_, dist_coeffs_, input_image.size());
        aruco::CameraParameters cam_params;  
       
        cam_params.setParams(camera_matrix_, dist_coeffs_, input_image.size());
        
        std::cout << " camera params " << cam_params << std::endl;
        
        //det=markerdetector.detect(input_image, cam_params, -1 , false , false);
        det = markerdetector.detect(input_image);

        if (det.empty()) {
            RCLCPP_WARN(this->get_logger(), "No ArUco markers detected.");
            return;
        }
        
        
        // Estimate pose of the first detected marker
        aruco::Marker marker = det[0];
        marker.calculateExtrinsics( 0.1, cam_params, true );
        //std::cout << " det : " << det[0] << std::endl;
        std::cout << "rvec " << marker.Rvec << std::endl;
        std::cout << "tvec " << marker.Tvec << std::endl;
        std::cout << "coordinate centro " << marker.getCenter() << std::endl;

        
        //std::cout << "size camera " << input_image.size() << std::endl;

        cv::Mat Rot_mat;
        cv::Rodrigues(marker.Rvec, Rot_mat); //trasforma il formato di Rodrigues a matrice di rotazione 
        //std::cout << "matrice di rot " << Rot_mat << std::endl;



       
       /*
       //DEBUG PER LA DETECTION
       for (const auto& marker : det) {           
           marker.draw(input_image, cv::Scalar(0, 255, 0), 2); // Disegna il marker
        }
        
        // Pubblicazione dell'immagine processata su "/videocamera_processata"
       cv::imshow("keypoints", input_image ); 
       cv::waitKey(3);
        */
        
       
     }
     else{
        std::cout << " Parameters not available "<< std::endl;
     }
       
    }
    
    
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        
        if(!camera_state_available_){
             // Store camera intrinsics
            camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data()).clone();
            dist_coeffs_ = cv::Mat(1, 5, CV_64F, (void*)msg->d.data()).clone();
            RCLCPP_INFO(this->get_logger(), "Camera parameters received.");
            std::cout << " camera matrix " << camera_matrix_ << std::endl;
            std::cout << " coeff " << dist_coeffs_ << std::endl;

            camera_state_available_=true;

        }
       
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Mat input_image;
    aruco::MarkerDetector markerdetector;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    aruco::MarkerPoseTracker pose_tracker_;
    bool camera_state_available_=false;




    //robot

    std::shared_ptr<KDLRobot> robot_;

    int iteration_;
    bool joint_state_available_;
    double t_;


};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionController>());
    rclcpp::shutdown();
    return 0;
}

