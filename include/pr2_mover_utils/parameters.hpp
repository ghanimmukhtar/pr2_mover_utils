#ifndef __PARAMETERS_HPP__
#define __PARAMETERS_HPP__
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <ros/callback_queue.h>
#include <string>
#include <math.h>
#include <Eigen/Core>

#include <boost/bind.hpp>
#include <memory>

#include <vector>

#include <geometry_msgs/PoseStamped.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group/move_group_context.h>

#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <std_srvs/Empty.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pr2_mover_utils/move_pr2_arm.h>

struct Parameters {
    std::vector<std::string> arm_joints_names_left = {"l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint",
                                                 "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"};

    std::vector<std::string> arm_joints_names_right = {"r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint",
                                                 "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"};
    sensor_msgs::JointState my_joint_state;
    geometry_msgs::Pose l_eef_pose, r_eef_pose;
    geometry_msgs::PoseStamped pose_target;
    Eigen::VectorXd l_eef_rpy_pose, r_eef_rpy_pose;
    Eigen::Vector3d l_eef_position, r_eef_position;
    Eigen::Vector3d l_eef_rpy_orientation, r_eef_rpy_orientation;

    XmlRpc::XmlRpcValue planner_parameters;

    moveit_msgs::GetMotionPlanRequest final_motion_request;
    moveit_msgs::GetMotionPlanResponse final_motion_response;

    moveit_msgs::ExecuteKnownTrajectoryRequest execute_traj_req;
    moveit_msgs::ExecuteKnownTrajectoryResponse execute_traj_res;

    std_srvs::Empty::Request empty_octomap_request;
    std_srvs::Empty::Response empty_octomap_response;

    std::unique_ptr<robot_model_loader::RobotModelLoader> robot_model_loader;
    robot_model::RobotModelPtr robot_model;

    moveit_msgs::GetPlanningScene::Request ps_req;
    moveit_msgs::GetPlanningScene::Response ps_res;
    moveit_msgs::PlanningScene ps_msg;
    bool add_octomap_to_acm = false;
    double approach_radius = 0;
};

class Data_config{

public:
    Parameters params;

    Data_config(){

    }

    //// Getters
    //left and right grippers pose variables getters
    geometry_msgs::Pose& get_eef_pose(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            return params.l_eef_pose;
        else
            return params.r_eef_pose;
    }

    geometry_msgs::PoseStamped& get_pose_target(){
        return params.pose_target;
    }

    std::vector<std::string>& get_pr2_left_arm_joints_names(){
        return params.arm_joints_names_left;
    }

    std::vector<std::string>& get_pr2_right_arm_joints_names(){
        return params.arm_joints_names_right;
    }

    sensor_msgs::JointState& get_joint_state(){
        return params.my_joint_state;
    }

    Eigen::VectorXd& get_eef_rpy_pose(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            return params.l_eef_rpy_pose;
        else
            return params.r_eef_rpy_pose;
    }

    Eigen::Vector3d& get_eef_position(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            return params.l_eef_position;
        else
            return params.r_eef_position;
    }

    Eigen::Vector3d& get_eef_rpy_orientation(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            return params.l_eef_rpy_orientation;
        else
            return params.r_eef_rpy_orientation;
    }

    XmlRpc::XmlRpcValue& get_planner_parameters(){
        return params.planner_parameters;
    }

    moveit_msgs::GetMotionPlanRequest& get_motion_request(){
        return params.final_motion_request;
    }

    moveit_msgs::GetMotionPlanResponse& get_motion_response(){
        return params.final_motion_response;
    }

    moveit_msgs::ExecuteKnownTrajectoryRequest& get_motion_execute_request(){
        return params.execute_traj_req;
    }

    moveit_msgs::ExecuteKnownTrajectoryResponse& get_motion_execute_response(){
        return params.execute_traj_res;
    }

    std_srvs::Empty::Request& get_empty_octomap_request(){
        return params.empty_octomap_request;
    }

    std_srvs::Empty::Response& get_empty_octomap_response(){
        return params.empty_octomap_response;
    }

    moveit_msgs::GetPlanningScene::Request& get_ps_request(){
        return params.ps_req;
    }

    moveit_msgs::GetPlanningScene::Response& get_ps_response(){
        return params.ps_res;
    }

    moveit_msgs::PlanningScene& get_ps_msg(){
        return params.ps_msg;
    }

    bool& get_adding_octomap_to_acm(){
        return params.add_octomap_to_acm;
    }

    double get_approach_radius(){
        return params.approach_radius;
    }

    //// Setters
    //left and right grippers pose variables getters
    void set_robot_model_loader(){
        params.robot_model_loader.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    }

    void set_robot_model(){
        params.robot_model = params.robot_model_loader->getModel();
    }

    void set_joint_state(const sensor_msgs::JointState::ConstPtr& jo_state){
        params.my_joint_state = *jo_state;
    }

    void set_eef_pose(geometry_msgs::Pose& eef_pose, const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            params.l_eef_pose = eef_pose;
        else
            params.r_eef_pose = eef_pose;
    }

    void set_pose_target(geometry_msgs::PoseStamped& pose_target){
        params.pose_target = pose_target;
    }

    void set_eef_rpy_pose(Eigen::VectorXd& eef_rpy_pose, const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            params.l_eef_rpy_pose = eef_rpy_pose;
        else
            params.r_eef_rpy_pose = eef_rpy_pose;
    }

    void set_eef_position(Eigen::Vector3d& eef_position, const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            params.l_eef_position = eef_position;
        else
            params.r_eef_position = eef_position;
    }

    void set_eef_rpy_orientation(Eigen::Vector3d& eef_rpy_orientation, const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            params.l_eef_rpy_orientation = eef_rpy_orientation;
        else
            params.r_eef_rpy_orientation = eef_rpy_orientation;
    }
    void set_planner_parameters(XmlRpc::XmlRpcValue& planner_params){
        params.planner_parameters = planner_params;
    }

    void set_motion_request(moveit_msgs::GetMotionPlanRequest& motion_plan_request){
        params.final_motion_request = motion_plan_request;
    }

    void set_motion_response(moveit_msgs::GetMotionPlanResponse& motion_plan_response){
        params.final_motion_response = motion_plan_response;
    }

    void set_ps_request(moveit_msgs::GetPlanningScene::Request req){
        params.ps_req = req;
    }

    void set_ps_response(moveit_msgs::GetPlanningScene::Response res){
        params.ps_res = res;
    }

    void set_ps_msg(moveit_msgs::PlanningScene msg){
        params.ps_msg = msg;
    }

    void set_adding_octomap_to_acm(bool add){
        params.add_octomap_to_acm = add;
    }

    void set_approach_radius(double radius){
        params.approach_radius = radius;
    }
};

#endif
