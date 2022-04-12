// server.cpp
// Author: Erica Hwang
// Referenced from robot_interface.cpp by Gopika Ajaykumar

#include "ros/ros.h"
#include <string.h>

// Move-It Actions and Messages
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/RobotTrajectory.h>

// Message types
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include "std_msgs/String.h"

#include <control_msgs/GripperCommandAction.h>

std::vector<std::vector<double> > waypoints; // Stores waypoints (list of respective joint positions)
std::vector<float> gripper_positions;       // Stores the gripper positions associated with each waypoint; corresponds to waypoints vector

// MoveIt!
boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> move_group;   // Shared pointer to MoveGroupInterface 
boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group;  
boost::scoped_ptr<actionlib::SimpleActionClient<control_msgs::GripperCommandAction>> gripper_ac;

static const std::string PLANNING_GROUP = "manipulator";
moveit_msgs::RobotTrajectory trajectory_msg;
const float MAX_TIME = 120.0;   // Maximum planning time
const int MAX_ATTEMPTS = 10;    // Maximum attempts allowed for replanning
const float VEL_SCALE = 0.1;    
const float ACC_SCALE = 0.1;
std::string planner_plugin_name;
std::string ns;

bool lastTeachModeEnabled = false;
bool teachModeEnabled = false;

int getWaypointCount()
{
	int size = gripper_positions.size();
	ROS_INFO("Waypoint Count: [%d]", size);

	return size;
}

void getWaypoints() {
	std::string waypointString = "";
	for (int i = 0; i < waypoints.size(); i++) {
		waypointString = "[";

		std::vector<double> waypoint = waypoints[i];
		for (int j = 0; j < waypoint.size(); j++) {
			waypointString += " ";
			double waypointVal = waypoint[j];
			waypointString += std::to_string(waypointVal);

			if (j != waypoint.size() - 1) {waypointString += ",";}
		}

		waypointString += " ]";
		ROS_INFO("%s", waypointString.c_str());
	}	
}

void addWaypointCallback(const std_msgs::String::ConstPtr& msg)
{
	if (msg->data.c_str() != "") {
		int index = std::stoi(msg->data.c_str());
		if (index < 0) { // Add to end
			ROS_INFO("Insert At End: [%d]", getWaypointCount());
			waypoints.push_back(move_group->getCurrentJointValues());
			gripper_positions.push_back(0.0);
		} else { // Positive input
			ROS_INFO("Insert Index: [%d]", index);

			// TODO: add error checking for out of bounds insertions
			waypoints.insert(waypoints.begin() + index, move_group->getCurrentJointValues());
			gripper_positions.insert(gripper_positions.begin() + index, 0.0);
		}
	} else { //Add to end as default
		ROS_INFO("Insert At End: [%d]", getWaypointCount());

		waypoints.push_back(move_group->getCurrentJointValues());
		gripper_positions.push_back(0.0);
	}

	// send back waypoint collection size?
	getWaypointCount();
	getWaypoints();
}

void deleteWaypointCallback(const std_msgs::String::ConstPtr& msg)
{
	if (getWaypointCount()) { // only if we have waypoints stored
		int index = std::stoi(msg->data.c_str());
		ROS_INFO("Delete Index: [%d]", index);

		// Invalid deletion index
		if (index > getWaypointCount() - 1) { 
			ROS_INFO("INVALID DELETION INDEX!!");
		} 
		// Valid deletion index
		else{ // Valid deletion index
		waypoints.erase(waypoints.begin() + index);
		gripper_positions.erase(gripper_positions.begin() + index);

		// send back waypoint collection size?
		getWaypointCount();
		}
		
	}
	getWaypoints();
	
}

void executeInspect(int indexToInspect) {
	bool success;
    move_group->setPlanningTime(MAX_TIME);
    move_group->setMaxVelocityScalingFactor(VEL_SCALE);
    move_group->setMaxAccelerationScalingFactor(ACC_SCALE);
    move_group->setGoalJointTolerance(0.001);
    move_group->setJointValueTarget(waypoints[indexToInspect]);
    move_group->setStartStateToCurrentState();

    success = (move_group->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    success = true;
    if (!success)
    {
        //feedback.status = "Unable to move to specified inspection point";
        //as->publishFeedback(feedback);
        //as->setAborted();
        ROS_INFO("Aborting goal since could not move to inspection point");
        return;
    }

    move_group->clearPoseTargets();
}

void executeWaypointsCallback(const std_msgs::String::ConstPtr& msg)
{
	getWaypointCount();
	ROS_INFO("Executing!");

	for (int i = 0; i < waypoints.size(); i++) {
		executeInspect(i);
	}

	ROS_INFO("Done");

	//const robot_state::JointModelGroup* gripper_model_group = gripper_move_group.getCurrentState()->getJointModelGroup("gripper");
    //std::vector<moveit_msgs::RobotTrajectory>::iterator it = trajs_msg.trajectories.begin();
    //moveit::core::RobotStatePtr current_gripper_state;
    //std::vector<double> gripper_joint_group_positions;
}

// TODO: Create topic on Android side
void teachModeCallback(const std_msgs::String::ConstPtr& msg)
{
	teachModeEnabled = !teachModeEnabled;
}

// TODO: Create topic on Android side
void gripCallback(const std_msgs::String::ConstPtr& msg)
{
	//actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_ac("icl_gripper/gripper_cmd", true);

	float amtToGrip = std::stof(msg->data.c_str());

	const robot_state::JointModelGroup* gripper_model_group = gripper_move_group->getCurrentState()->getJointModelGroup("gripper");
	moveit::core::RobotStatePtr current_gripper_state;
	current_gripper_state = gripper_move_group->getCurrentState();
	std::vector<double> gripper_joint_group_positions;
    current_gripper_state->copyJointGroupPositions(gripper_model_group, gripper_joint_group_positions);

    control_msgs::GripperCommandGoal goal;
    goal.command.position = amtToGrip;
    gripper_ac->sendGoal(goal);
    ros::Duration(1.5).sleep();
    current_gripper_state = gripper_move_group->getCurrentState();
    current_gripper_state->copyJointGroupPositions(gripper_model_group, gripper_joint_group_positions);
}

// make new callback to also publish waypointcount

int main(int argc, char **argv) {
	ros::init(argc, argv, "server");
	ros::NodeHandle node_handle;

	ros::AsyncSpinner spinner(2); 
    spinner.start();

	move_group.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
	gripper_move_group.reset(new moveit::planning_interface::MoveGroupInterface("gripper"));
	gripper_ac.reset(new actionlib::SimpleActionClient<control_msgs::GripperCommandAction>("icl_gripper/gripper_cmd", true));

	if (!node_handle.getParam("move_group/planning_plugin", planner_plugin_name))
    {
        ROS_FATAL_STREAM("Could not find planner plugin name");
    }

	ros::Subscriber addWayPointSub = node_handle.subscribe("addWayPoint", 1000, addWaypointCallback);
  	ros::Subscriber delWayPointSub = node_handle.subscribe("deleteWayPoint", 1000, deleteWaypointCallback);
  	//ros::Subscriber getWaypointCount = node_handle.subscribe("getWaypointCount", 1000, sendWaypointCount);
  	ros::Subscriber executeWaypoints = node_handle.subscribe("executeCallback", 1000, executeWaypointsCallback);
  	ros::Subscriber teachModeSub = node_handle.subscribe("teachModeCallback", 1000, teachModeCallback);
  	ros::Subscriber gripSub = node_handle.subscribe("gripCallback", 1000, gripCallback);

  	ros::Publisher teachModePub = node_handle.advertise<std_msgs::String>("/icl_ur5/ur_driver/URScript", 1000);
  	std_msgs::String teachModeMsg;

  	while(ros::ok())
  	{
  		lastTeachModeEnabled = teachModeEnabled;
  		ros::spinOnce();
	  	if (!lastTeachModeEnabled && teachModeEnabled)
	  	{
	  		teachModeMsg.data = "set robotmode freedrive";
	  		teachModeMsg.data = "def myProg():\n";
	        teachModeMsg.data += "\twhile (True):\n";
	        teachModeMsg.data += "\t\tfreedrive_mode()\n";
	        teachModeMsg.data +="\t\tsync()\n";
	        teachModeMsg.data += "\tend\n";
	        teachModeMsg.data +="end\n";
	        teachModePub.publish(teachModeMsg);
	        ROS_INFO("Teach mode enabled");
	  	}

	  	if(lastTeachModeEnabled && !teachModeEnabled)
	  	{
	  		//teachModeMsg.data = "set robotmode run";
	        teachModeMsg.data = "def myProg():\n";
	        teachModeMsg.data += "\twhile (True):\n";
	        teachModeMsg.data += "\t\tend_freedrive_mode()\n";
	        teachModeMsg.data +="\t\tsleep(0.5)\n";
	        teachModeMsg.data += "\tend\n";
	        teachModeMsg.data +="end\n";
	        teachModePub.publish(teachModeMsg);
	        ROS_INFO("Teach mode disabled");
	    }
	}

    ros::waitForShutdown();
	return 0;
}

/* 
close code
const robot_state::JointModelGroup* gripper_model_group = gripper_move_group.getCurrentState()->getJointModelGroup("gripper");
        moveit::core::RobotStatePtr current_gripper_state;
        std::vector<double> gripper_joint_group_positions;
        current_gripper_state = gripper_move_group.getCurrentState();
        current_gripper_state->copyJointGroupPositions(gripper_model_group, gripper_joint_group_positions);

        // Hard-coded close value, update as needed (or use F/T sensor)
        if (my_abs(gripper_joint_group_positions[0] - 0.8f) > 5e-3)  // If gripper needs to be moved
        {
            control_msgs::GripperCommandGoal goal;
            goal.command.position = 0.8;
            feedback.status = "Closing gripper";
            as->publishFeedback(feedback);
            gripper_ac.sendGoal(goal);
            ros::Duration(1.5).sleep();
            current_gripper_state = gripper_move_group.getCurrentState();
            current_gripper_state->copyJointGroupPositions(gripper_model_group, gripper_joint_group_positions);
            feedback.status = "Closed gripper";
            as->publishFeedback(feedback);
        }

 open code
 const robot_state::JointModelGroup* gripper_model_group = gripper_move_group.getCurrentState()->getJointModelGroup("gripper");
        moveit::core::RobotStatePtr current_gripper_state;
        std::vector<double> gripper_joint_group_positions;
        current_gripper_state = gripper_move_group.getCurrentState();
        current_gripper_state->copyJointGroupPositions(gripper_model_group, gripper_joint_group_positions);

        // Hard-coded open value, update as needed (or use F/T sensor)
        if (my_abs(gripper_joint_group_positions[0] - 0.1f) > 5e-2) 
        {
            control_msgs::GripperCommandGoal goal;
            goal.command.position = 0.1f;
            feedback.status = "Opening gripper";
            as->publishFeedback(feedback);
            gripper_ac.sendGoal(goal);
            ros::Duration(1.5).sleep();
            current_gripper_state = gripper_move_group.getCurrentState();
            current_gripper_state->copyJointGroupPositions(gripper_model_group, gripper_joint_group_positions);
            if (my_abs(gripper_joint_group_positions[0] - 0.1f) > 5e-2)
            {
                feedback.status = "Failed to open gripper";
                as->publishFeedback(feedback);
                as->setAborted();
                return;
            }
            feedback.status = "Opened gripper";
            as->publishFeedback(feedback);
*/