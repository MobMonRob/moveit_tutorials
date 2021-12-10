#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


const double tau = 2 * M_PI;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;


  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();

  visual_tools.trigger();

  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());


  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit_msgs::MotionPlanRequest req;
  std::vector<double> joint_group_positions;
  std::vector<double> joint_group_positions2;
  std::vector<double> joint_group_positions3;
  std::vector<double> joint_group_positions4;
  
  //moveit_msg::MotionPlanResponse res;

  req.planner_id = "PTP";
  req.group_name = PLANNING_GROUP;
  move_group_interface.setPlannerId(req.planner_id);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //geometry_msgs::PoseStamped goal;
  //goal.header.frame_id = "base_link";
  //goal.pose.orientation.x = 1;
  //goal.pose.position.x = 0.30;
  //goal.pose.position.y = 0.38;
  //goal.pose.position.z = 0.69;
  //move_group_interface.setPoseTarget(goal);

  //moveit_msgs::Constraints path_constraints;
  //path_constraints.name = "interim";
  //moveit_msgs::PositionConstraint pos_constraint;
  //goal.pose.position.x = 0.13;
  //pos_constraint.constraint_region.primitive_poses.push_back(goal.pose);
  //pos_constraint.link_name= "wrist_3_link";
  //pos_constraint.header.frame_id = "base_link";
  //path_constraints.position_constraints.push_back(pos_constraint);
  //move_group_interface.setPathConstraints(path_constraints);
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  for(int i=0; i<4; i++)
  {
    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();

    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    joint_group_positions[0] = -90.24 * 3.1415 / 180;  
    joint_group_positions[1] = -111.72 * 3.1415 / 180;  
    joint_group_positions[2] = -78.96 * 3.1415 / 180;  
    joint_group_positions[3] = -75.76 * 3.1415 / 180;  
    joint_group_positions[4] = 91.52 * 3.1415 / 180;  
    //joint_group_positions[5] = -126.97 * 1415 / 180;  

    move_group_interface.setJointValueTarget(joint_group_positions);

    move_group_interface.setMaxVelocityScalingFactor(0.25);
    move_group_interface.setMaxAccelerationScalingFactor(0.25);

    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");

    move_group_interface.move();

    move_group_interface.setStartState(*move_group_interface.getCurrentState());

    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions2);


    joint_group_positions2[0] = -168.07 * 3.1415 / 180;  
    joint_group_positions2[1] = -108.79 * 3.1415 / 180;  
    joint_group_positions2[2] = -78.97* 3.1415 / 180;  
    joint_group_positions2[3] = -37.36 * 3.1415 / 180; 
    joint_group_positions2[4] = 70.52 * 3.1415 / 180;  
    //joint_group_positions2[5] = -255.27 * 1415 / 180;  

    move_group_interface.setJointValueTarget(joint_group_positions2);


    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    move_group_interface.move();


    move_group_interface.setStartState(*move_group_interface.getCurrentState());


    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions3);


    joint_group_positions3[0] = -229.55 * 3.1415 / 180;  
    joint_group_positions3[1] = -89.22 * 3.1415 / 180;  
    joint_group_positions3[2] = -102.1* 3.1415 / 180;  
    joint_group_positions3[3] = 11.69 * 3.1415 / 180;  
    joint_group_positions3[4] = 73.62 * 3.1415 / 180;  
    //joint_group_positions3[5] = -312.78 * 1415 / 180;  

    move_group_interface.setJointValueTarget(joint_group_positions3);




    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (joint space goal) %s", success ? "" : "FAILED");

    move_group_interface.move();

    move_group_interface.setStartState(*move_group_interface.getCurrentState());

    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions4);

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    joint_group_positions4[0] = -355.54 * 3.1415 / 180; 
    joint_group_positions4[1] = -118.64 * 3.1415 / 180; 
    joint_group_positions4[2] = -79.15 * 3.1415 / 180;  
    joint_group_positions4[3] = -181.72 * 3.1415 / 180;  
    joint_group_positions4[4] = -121.70 * 3.1415 / 180;  
    //joint_group_positions4[5] = -293.33 * 1415 / 180;  

    move_group_interface.setJointValueTarget(joint_group_positions4);

    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (joint space goal) %s", success ? "" : "FAILED");

    move_group_interface.move();
  }

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
