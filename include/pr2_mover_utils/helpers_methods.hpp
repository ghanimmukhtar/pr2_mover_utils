#include <pr2_mover_utils/parameters.hpp>

namespace pr2_helpers_methods{
/*get baxter left/right eef pose with orientation expressed as RPY
 * input: pose as geometry msgs of the baxter eef (including the ), and the name of the gripper
 * return: nothing but set the corresponding variable in the Data_config class
 * */
void locate_eef_pose(geometry_msgs::Pose &eef_feedback, Data_config& parameters, const std::string gripper);

/* set the motion plan request as delivered from any node wants to use the class BAXTER_mover
 * input: request and response to the service baxter_mover_utils::move_baxter_arm, and Data_config class
 * return: nothing but set the corresponding variable in the Data_config class
 * */
void prepare_motion_request(pr2_mover_utils::move_pr2_arm::Request &req,
                            pr2_mover_utils::move_pr2_arm::Response &res,
                            Data_config& params);

}
