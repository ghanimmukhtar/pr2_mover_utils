#ifndef _PR2_MOVER_HPP
#define _PR2_MOVER_HPP

#include <pr2_mover_utils/helpers_methods.hpp>

namespace pr2_mover {
class PR2_Mover {
public:
    typedef std::shared_ptr<PR2_Mover> Ptr;
    typedef const std::shared_ptr<PR2_Mover> ConstPtr;

    PR2_Mover(ros::NodeHandle& nh){
        //nh_.reset(new ros::NodeHandle(nh, "pr2_mover"));
        init(nh);

    }

    void init(ros::NodeHandle& nh);
    bool move_pr2_arm_cb(pr2_mover_utils::move_pr2_arm::Request& req,
                            pr2_mover_utils::move_pr2_arm::Response& res);


    //call back that register crustcrawler joint states
    void joint_state_Callback(const sensor_msgs::JointState::ConstPtr& joint_state_feedback){
        //ROS_INFO_STREAM("PR2_MOVER : Size of joint state is: " << joint_state_feedback->position.size());
        global_parameters.set_joint_state(joint_state_feedback);
    }

    void call_service_get_ps(){
        global_parameters.get_ps_request().components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
        _get_planning_scene->call(global_parameters.get_ps_request(), global_parameters.get_ps_response());
        if(global_parameters.get_adding_octomap_to_acm()){
            global_parameters.get_ps_response().scene.allowed_collision_matrix.default_entry_names.push_back("<octomap>");
            global_parameters.get_ps_response().scene.allowed_collision_matrix.default_entry_values.push_back(true);
        }
        else{
            global_parameters.get_ps_response().scene.allowed_collision_matrix.default_entry_names.clear();
            global_parameters.get_ps_response().scene.allowed_collision_matrix.default_entry_values.clear();
        }
    }

    void publish_psm_msg(){
        global_parameters.get_ps_msg().is_diff = true;
        global_parameters.get_ps_msg().allowed_collision_matrix = global_parameters.get_ps_response().scene.allowed_collision_matrix;
        _psm_pub->publish(global_parameters.get_ps_msg());
    }

    void publish_psm_msg(moveit_msgs::PlanningScene planning_scene){
        _psm_pub->publish(planning_scene);
    }

    Data_config global_parameters;
    std::shared_ptr<moveit::planning_interface::MoveGroup> group, secondary_group;

private:
    std::unique_ptr<ros::AsyncSpinner> _my_spinner;
    std::unique_ptr<ros::ServiceServer> _pr2_mover;
    std::unique_ptr<ros::ServiceClient> _get_motion_plan;
    std::unique_ptr<ros::ServiceClient> _execute_motion_plan;
    std::unique_ptr<ros::ServiceClient> _clear_octomap;
    std::unique_ptr<ros::ServiceClient> _get_planning_scene;
    std::unique_ptr<ros::Publisher> _psm_pub;
    std::string _planner_id;
    std::unique_ptr<ros::Subscriber> _sub_joint_state_msg;
    XmlRpc::XmlRpcValue _planner_parameters;

    ros::NodeHandlePtr _nh;
};


}

#endif //_PR2_MOVER_HPP
