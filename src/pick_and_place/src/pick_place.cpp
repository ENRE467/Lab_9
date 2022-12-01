#include "../include/pick_place.hpp"



PickandPlace::PickandPlace(ros::NodeHandle *nodehandle): PP{*nodehandle} {
    this->pick_detected = false;
    fiducial_sub = PP.subscribe("/logitech_webcam/fiducial_transforms", 1, &PickandPlace::Fiducial_Callback, this);
    pick_sub = PP.subscribe("/aruco_tf/picktopic", 1, &PickandPlace::Pick_Callback, this);
}

void PickandPlace::Pick_Callback(const geometry_msgs::Pose::ConstPtr& pick) {
  if(this->pick_detected){
    this->Pick.position.x = pick->position.x;
    this->Pick.position.y = pick->position.y;
    this->Pick.position.z = pick->position.z + 0.2;
    // this->Pick.orientation.x = pick->orientation.x;
    // this->Pick.orientation.y = pick->orientation.y;
    // this->Pick.orientation.z = pick->orientation.z;
    // this->Pick.orientation.w = pick->orientation.w;
    this->Pick.orientation.x = 1.0;
    this->Pick.orientation.y = 0.0;
    this->Pick.orientation.z = 0.0;
    this->Pick.orientation.w = 0.0;
  }
}

void PickandPlace::Fiducial_Callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
  
  if(!msg->transforms.empty()){
    for(int i = 0; i < msg->transforms.size(); i++){
        if(msg->transforms[i].fiducial_id == 1){
            this->pick_detected = true;
            break;
        }
        else{
            this->pick_detected = false;
        }
    }
  }
}

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "pickplace");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Create PlanningOptions
  MoveitPlanning::PlanningOptions planning_options =
  MoveitPlanning::PlanningOptions();
  planning_options.num_attempts = 10;
  planning_options.allow_replanning = true;
  planning_options.set_planning_time = 30.0;
  planning_options.goal_position_tolerance = 0.01;
  planning_options.goal_orientation_tolerance = 0.01;
  planning_options.goal_joint_tolerance = 0.01;
  planning_options.velocity_scaling_factor = 0.1;
  planning_options.acceleration_scaling_factor = 0.1;

    // Create instance of MoveGroupInterface for given joint group
  moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

  ros::NodeHandle nh;
  PickandPlace pickplace(&nh);
  ros::Rate r(10);

  while(ros::ok()){
    ros::spinOnce();
    // ROS_INFO("Looking for Marker");
    if(pickplace.pick_detected) break;
    r.sleep();
  }

  // Pre-grasp
  moveit::planning_interface::MoveGroupInterface::Plan pick_plan;

  int flag = 0;
  bool pick_plan_success;
  std::string reference_frame = "base_link";

  ROS_INFO("Planning to");
  ROS_INFO("x: %f",pickplace.Pick.position.x);
  ROS_INFO("y: %f",pickplace.Pick.position.y);
  ROS_INFO("z: %f",pickplace.Pick.position.z);

  pick_plan_success = ArmController::planToPoseTarget(planning_options,arm_move_group,pickplace.Pick,reference_frame,pick_plan);

  if(pick_plan_success){
        ROS_INFO("pick plan succeeded");
        ros::Duration(1.0).sleep();
        flag = 1;

        arm_move_group.execute(pick_plan);

    }
    else{
        ROS_INFO("pick plan failed");
        flag = 0;
  }
  
  if(flag){
    // Create instance of cartesian plan (grasp)
    moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;

    // Get the start Pose
    geometry_msgs::Pose start_pose = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose = start_pose;
    end_pose.position.z -= 0.2;

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(end_pose);

    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(start_pose, waypoints, reference_frame, arm_move_group);

    arm_move_group.execute(trajectory);

  }

  
}


