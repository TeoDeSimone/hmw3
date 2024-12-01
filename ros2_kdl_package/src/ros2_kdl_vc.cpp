// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
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


//camera
#include "aruco/markerdetector.h"
#include "aruco/aruco.h"
#include "aruco/posetracker.h"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_vc"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare cmd_interface parameter (position, velocity, effort)
            declare_parameter("cmd_interface", "velocity"); // defaults to "position"
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            //controllo sull'ammissibilità del parametro cmd_interface_
            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }
            
            

            while(!(control_technique_== "JS" || control_technique_== "OS"))  //joint space inverse dynamic controller o operational space inverse dynamics controller            
            {
                std::cout<<"insert inverse dynamic controller approach:"<<std::endl<<"joint space(JS)/ operational space(OS):";
                std::cin>>control_technique_;
                if(!(control_technique_=="JS"||control_technique_=="OS"))
                    RCLCPP_INFO(get_logger(),"Selected control technique is not valid!");

            }

/*
            while(!(trajectory_type_=="circ"||trajectory_type_=="lin"))
            {
                std::cout<<"insert desired trajectory (circ/lin):";
                std::cin>>trajectory_type_;
                if(!(trajectory_type_=="circ"||trajectory_type_=="lin"))
                    RCLCPP_INFO(get_logger(),"Selected trajectory type is not valid!");
            }
*/
            trajectory_type_="lin";

            while(!(trajectory_profile_=="trap"||trajectory_profile_=="cubic"))                
            {
                std::cout<<"insert desired trajectory profile (trap/cubic):";
                std::cin>>trajectory_profile_;
                if(!(trajectory_profile_=="trap"||trajectory_profile_=="cubic"))
                    RCLCPP_INFO(get_logger(),"Selected trajectory profile is not valid!");

            }

            if(control_technique_=="JS")
            {
                kp_=1;
                kd_=1;
                kpo_=1;  //unused
                kdo_=1;  //unused

            }    


            if(control_technique_=="OS")
            {
                kp_=50;
                kd_=7;
                kpo_=50;  
                kdo_=50;  

            }    
        
            iteration_ = 0;//this will take account of number of cmd_publisher iterations
            t_ = 0;
            joint_state_available_ = false; 
            aruco_pose_detected_=false;

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) 
            {
                if (!rclcpp::ok()) 
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree))
            {
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj);
            joint_torques_.resize(nj); 
            jnt_vel_.resize(nj);
            jnt_acc_.resize(nj);
            jnt_pos_.resize(nj);

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_)
            {
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }


           
 
            
                
                // Update KDLrobot object
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
                KDL::Frame f_T_ee = KDL::Frame::Identity();
                robot_->addEE(f_T_ee);
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                // Compute EE frame
                init_cart_pose_ = robot_->getEEFrame();
                // std::cout << "The initial EE pose is: " << std::endl;  
                // std::cout << init_cart_pose_ <<std::endl;

                // Compute IK in order to know the initial position of the joints
                KDL::JntArray q(nj);
                robot_->getInverseKinematics(init_cart_pose_, q);
                // std::cout << "The inverse kinematics returned: " <<std::endl; 
                // std::cout << q.data <<std::endl;

                // Initialize controller
                KDLController controller_(*robot_);

                // EE's trajectory initial position (just an offset)
                Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data));

                // EE's trajectory end position(just opposite y) and wakeup_end_position (a rest position from which the robot will execute the required trajectory)
                Eigen::Vector3d end_position;  



                //Subscrivber to aruco marker pose
                //geometry_msgs/msg/PoseStamped
                aruco_mark_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/aruco_single/pose", 10, std::bind(&Iiwa_pub_sub::pose_subscriber, this, std::placeholders::_1));

                // Wait for the joint_state topic
                while(aruco_pose_detected_<1)
                {
                    std::cout<<aruco_pose_detected_<<'\n';
                    RCLCPP_INFO(this->get_logger(), "No aruco pose received yet! ...");
                    rclcpp::spin_some(node_handle_);
                }


                //final point of the linear trajectory
                end_position<<aruco_frame_bf_.p.data[0],aruco_frame_bf_.p.data[1]+0.05,aruco_frame_bf_.p.data[2]; 

                std::cout<<"ENDPOSITIONDISTOCAZZO :"<<end_position(0)<<" "<<end_position(1)<<" "<<end_position(2)<<"\n\n\n\n\n\n\n";

                std::cout<<"rotation_matrix: ["<<aruco_frame_bf_.M.data[0]<<' '<<aruco_frame_bf_.M.data[1]<<' '<<aruco_frame_bf_.M.data[2];
                std::cout<<'\n'<<aruco_frame_bf_.M.data[3]<<' '<<aruco_frame_bf_.M.data[4]<<' '<<aruco_frame_bf_.M.data[5];
                std::cout<<'\n' <<aruco_frame_bf_.M.data[6]<<' '<<aruco_frame_bf_.M.data[7]<<' '<<aruco_frame_bf_.M.data[8];

                
                
                // Plan trajectory 
                double traj_duration = 1.5, acc_duration = 0.5;
                
                planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position);//linear trajectory (sets the radius=0 as a flag)

             
                //create the publisher according to cmd_interface_value
                if(cmd_interface_ == "position")
                {
                    // Create cmd publisher
                    cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                
                    // Send joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) 
                    {
                        desired_commands_[i] = joint_positions_(i);
                    }
                }
                else if(cmd_interface_ == "effort")
                {
                    // Create cmd publisher
                    cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_torque_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                
                    // Send joint position commands
                    for (long int i = 0; i < joint_torques_.size(); ++i) 
                    {
                        desired_commands_[i] = joint_torques_(i);
                    }       
                    
                }
                else //velocity control
                { 
                    // Create cmd publisher
                    cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                
                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) 
                    {
                        desired_commands_[i] = joint_velocities_(i);
                    }
                }


                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
            
        }

    private:

        void pose_subscriber(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
        {

            if(aruco_pose_detected_<1)
            {
                //posa sta in pose_msg.pose;
                //trasl sta in pose_msg.pose.position
                //rotaz sta in pose_msg.pose.orientation

                KDL::Frame cameraframe = robot_->getEEFrame();  

                KDL::Rotation y_rotation = KDL::Rotation::RotY(M_PI);

                KDL::Vector aruco_trasl_vec_cf;
                Eigen::VectorXd aruco_quaternion_cf;

                aruco_quaternion_cf.resize(4);

                aruco_trasl_vec_cf[0] = pose_msg->pose.position.x;
                aruco_trasl_vec_cf[1] = pose_msg->pose.position.y;
                aruco_trasl_vec_cf[2] = pose_msg->pose.position.z;

                aruco_quaternion_cf[0] = pose_msg->pose.orientation.x;
                aruco_quaternion_cf[1] = pose_msg->pose.orientation.y;
                aruco_quaternion_cf[2] = pose_msg->pose.orientation.z;
                aruco_quaternion_cf[3] = pose_msg->pose.orientation.w;
       
                KDL::Frame aruco_frame_cf;
                aruco_frame_cf.M = KDL::Rotation::Quaternion(aruco_quaternion_cf[0], aruco_quaternion_cf[1], aruco_quaternion_cf[2], aruco_quaternion_cf[3]);

                aruco_frame_cf.M=aruco_frame_cf.M*y_rotation;

                aruco_frame_cf.p = aruco_trasl_vec_cf;
                aruco_frame_bf_= cameraframe * aruco_frame_cf;
                aruco_pose_detected_++;
                std::cout<<"AAAAAAAAAAAAAAAAAAAAAAAAA"<<aruco_frame_bf_.p.data[0]<<aruco_frame_bf_.p.data[1]<<aruco_frame_bf_.p.data[2]<<"AAAAAAAAAAAAAAAAAAAAAAA\n";
                std::cout<<"ARUCOPOSEDETECTED: "<<aruco_pose_detected_<<'\n';
            
            }


        }


        void cmd_publisher()
        {

            iteration_ = iteration_ + 1; //increment iteration_ counter

            // define trajectory
            double trajectory_time = 1.5; //the time to complete the requested trajectory
            
            int trajectory_len = 150;  
            int loop_rate = trajectory_len / trajectory_time;
            double dt = 1.0 / loop_rate;
            t_+=dt;

            unsigned int nj = robot_->getNrJnts();
            KDLController controller_(*robot_); 


//            if(t_ < total_time)
            //if (t_ < wakeup_time)
            {

                
                // Retrieve the trajectory point
                    
                if(t_<=trajectory_time)// retrive the points of the requested trajectory
                    p = planner_.compute_trajectory(t_,trajectory_profile_);
                else    //don't move
                    p = planner_.compute_trajectory(trajectory_time,trajectory_profile_);



                    //NB the compute_trajectory() function will compute 
                    //   the points using a linear or circular shape according 
                    //   to the used contructor: see lines circa 166-169  

                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();     

                std::cout<<"\ncartpos.p:"<<cartpos.p<<'\n';
                std::cout<<"cartpos.M:"<<cartpos.M<<'\n';


               
                // compute errors //distances between end effector and target position
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data)); 
                Eigen::Vector3d o_error = computeOrientationError(toEigen(aruco_frame_bf_.M), toEigen(cartpos.M));
                //std::cout << "The target is : " << p.pos[0] << p.pos[1] << p.pos[2] << std::endl;


                //compute desired frames
                //desired frame position
                KDL::Frame des_cartpos; des_cartpos.M = cartpos.M; des_cartpos.p = toKDL(p.pos);// + (toKDL(p.vel) + toKDL(1*error))*dt;
                
                //desired frame velocity
                KDL::Twist des_cartvel2; des_cartvel2.vel=toKDL(p.vel);// des_cartvel2.rot=toKDL(o_error);
                
                //desired frame acceleration
                KDL::Twist des_cartacc; des_cartacc.vel=toKDL(p.acc); //des_cartacc.rot initialized to zero





                if(cmd_interface_ == "position")
                {
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt; 

                    // Compute IK
                    robot_->getInverseKinematics(nextFrame, joint_positions_);
                }
                else if(cmd_interface_ == "effort")
                {
                    // Compute differential IK
                    //des_cartvel contains the desired cartesian velocties: << traslation, rotation
                    //includes also an error compensation (increase velocity if the cartesian error(angular or linear) is positive)
                    Vector6d des_cartvel; des_cartvel << p.vel + 10*error, o_error;

                    //save in temporary variable ex_joint_velocities in order to compute discrete derivative
                    KDL::JntArray ex_joint_velocities; 
                    ex_joint_velocities.resize(nj);
                    ex_joint_velocities.data=jnt_vel_.data; //now the last joint velocity (computed in last iteration) is still stored 
                                                                    // in joint_velocities. It's necessary to keep it in a temporary variable

                    jnt_vel_.data = pseudoinverse(robot_->getEEJacobian().data)*des_cartvel; //compute actual joint velocity
                    jnt_pos_.data = jnt_pos_.data + jnt_vel_.data*dt; //compute the discrete integration to obtain position
                    jnt_acc_.data = (jnt_vel_.data-ex_joint_velocities.data)/dt;//compute the discrete derivative to obtain acceleration

                    //compute joint_torques
                    if(control_technique_=="JS")
                        joint_torques_=controller_.idCntr(jnt_pos_,jnt_vel_,jnt_acc_,kp_,kd_);
                    else if(control_technique_=="OS")
                        joint_torques_=controller_.idCntr2(des_cartpos,des_cartvel2,des_cartacc,kp_,kpo_,kd_,kdo_);

    
                    std::cout<<"joint_torques_:"<<std::endl<<joint_torques_<<std::endl;
                    
                }
                else  //velocity control
                {

                    // Compute differential IK
                    Vector6d des_cartvel; des_cartvel << p.vel + 10*error, o_error;
                    
                    
                    joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*des_cartvel;
                    joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;
                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position")
                {
                    // Send joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) 
                    {
                        desired_commands_[i] = joint_positions_(i);
                    }
                }
                else if(cmd_interface_ =="effort")
                {
                    // Send effort position commands
                    for (long int i = 0; i < joint_torques_.size(); ++i) 
                    {
                        desired_commands_[i] = joint_torques_(i);
                    }
                }
                else
                {
                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) 
                    {
                        desired_commands_[i] = joint_velocities_(i);
                    }
                }

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
                // st d::cout << "EE pose is: " << robot_->getEEFrame() <<std::endl;  
                // std::cout << "Jacobian: " << robot_->getEEJacobian().data <<std::endl;
                // std::cout << "joint_positions_: " << joint_positions_.data <<std::endl;
                // std::cout << "joint_velocities_: " << joint_velocities_.data <<std::endl;
                // std::cout << "iteration_: " << iteration_ <<std::endl <<std::endl;
                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
            }
/*            else
            {
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) 
                {
                    desired_commands_[i] = 0;
                }
                
                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
  */
        }


         


        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg)
        {

            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
            // }

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++)
            {
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_accelerations_;
        KDL::JntArray jnt_pos_;
        KDL::JntArray jnt_vel_;
        KDL::JntArray jnt_acc_;
        Eigen::VectorXd joint_torques_;
        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;
        KDLPlanner wakeup_planner_;
        int iteration_;
        bool joint_state_available_;
        double t_;
        
        std::string cmd_interface_, trajectory_type_, trajectory_profile_, control_technique_;
        
        KDL::Frame init_cart_pose_;
        double kp_,kd_,kpo_,kdo_;
        trajectory_point p;


        //camera
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_mark_pose_sub_;

        KDL::Frame aruco_frame_bf_;
        int aruco_pose_detected_;
};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}
