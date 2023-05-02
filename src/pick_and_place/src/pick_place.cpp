#include "../include/pick_place.hpp"



PickandPlace::PickandPlace(ros::NodeHandle *nodehandle): PP{*nodehandle} {
    this->pick_detected = false;
    this->place_detected = false;
    fiducial_sub = PP.subscribe("/logitech_webcam/fiducial_transforms", 1, &PickandPlace::Fiducial_Callback, this);
    pick_sub = PP.subscribe("/aruco_tf/picktopic", 1, &PickandPlace::Pick_Callback, this);
    place_sub = PP.subscribe("/aruco_tf/placetopic", 1, &PickandPlace::Place_Callback, this);
}

void PickandPlace::Pick_Callback(const geometry_msgs::Pose::ConstPtr& pick) {
  if(this->pick_detected){
    this->Pick.position.x = pick->position.x;
    this->Pick.position.y = pick->position.y;
    this->Pick.position.z = pick->position.z;
    this->Pick.orientation.x = 1.0;
    this->Pick.orientation.y = 0.0;
    this->Pick.orientation.z = 0.0;
    this->Pick.orientation.w = 0.0;
  }
}

void PickandPlace::Place_Callback(const geometry_msgs::Pose::ConstPtr& place) {
  if(this->place_detected){
    this->Place.position.x = place->position.x;
    this->Place.position.y = place->position.y;
    this->Place.position.z = place->position.z;
    this->Place.orientation.x = 1.0;
    this->Place.orientation.y = 0.0;
    this->Place.orientation.z = 0.0;
    this->Place.orientation.w = 0.0;
  }
}

void PickandPlace::Fiducial_Callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
  
  if(!msg->transforms.empty()){
    for(int i = 0; i < msg->transforms.size(); i++){
        // change these fiducial ids to the ids of the markers used for pick and place
        if(msg->transforms[i].fiducial_id == 1){
            this->pick_detected = true;
            break;
        }
        else if(msg->transforms[i].fiducial_id == 2){
            this->place_detected = true;
        }
        else{
            this->pick_detected = false;
            this->place_detected = false;
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
    if(pickplace.pick_detected && pickplace.place_detected) break;
    r.sleep();
  }

  // Perform pre-grasp plan, i.e., move the gripper to above the pick position. 
  
  
  
  
  
  // Perform the grasp plan, i.e., move the gripper down over the block in cartesian path and close the gripper.
  
  
  
  
  
  // Perform the post-grasp plan, i.e., move the gripper up over the pick position such that the block comes off the table in a cartesian path.





  // Move the gripper to above the place position.





  // Move the gripper down such that the block is about to touch the table in a cartesian path and then open the gripper.





  // Move the gripper at some height above the placed block.
  




  
}


