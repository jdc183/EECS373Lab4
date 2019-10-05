#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include <stdlib.h>

// MoveIt header files
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Transformation header files
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

// Declaring a vector of a data type.
std::vector<osrf_gear::Order> order_vector;
osrf_gear::LogicalCameraImage logical_cam_image;


void camCallback(const osrf_gear::LogicalCameraImage& image){
	logical_cam_image = image;
}

void orderCallback(const osrf_gear::Order& order)
{
	// Clearing/initializing vector
	order_vector.clear();
	// Add information to the end of the vector
	order_vector.push_back(order);

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "subscriber_node");

	ros::NodeHandle n;

	ros::Subscriber sub_order = n.subscribe("ariac/orders", 1000, orderCallback);
	ros::Subscriber sub_logical_cam = n.subscribe("ariac/logical_camera", 1000, camCallback);
	// To declare the variable in this way where necessary in the code.
	std_srvs::Trigger begin_comp;

	// Create the service client
	ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

	if (!begin_client.exists()) {
    	ROS_INFO("Waiting for the competition to be ready...");
    	begin_client.waitForExistence();
    	ROS_INFO("Competition is now ready.");
  	}
  	ROS_INFO("Requesting competition start...");

  	begin_client.call(begin_comp);  // Call the start Service.

	if (!begin_comp.response.success) {  // If not successful, print out why.
	    ROS_WARN("Failed to start the competition: %s", begin_comp.response.message.c_str());
	} 
	else {
	    ROS_INFO("Competition started!");
	}

	// Processing Order part:
	osrf_gear::GetMaterialLocations srv;

	ros::ServiceClient get_location_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

	// Declare the transformation buffer to maintain a list of transformations
	tf2_ros::Buffer tfBuffer;
	// Instantiate a listener that listens to the tf and tf_static topics and to update the buffer.
	tf2_ros::TransformListener tfListener(tfBuffer);

	moveit::planning_interface::MoveGroupInterface move_group("manipulator");

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		if (!order_vector.empty())
		{
			//Indicate the type of the first object in the order and the storage units contain the object type.	
			const char* object_type = order_vector[0].kits[0].objects[0].type.c_str();
			//ROS_INFO("%s",object_type.c_str());

			srv.request.material_type = object_type;

			// Getting the location of the part
			if (get_location_client.call(srv))
			{
				//Showing all the storage units that currently hold the part
				for(int i = 0; i < srv.response.storage_units.size(); i++) {
					const char* location = srv.response.storage_units[i].unit_id.c_str();
					ROS_INFO("location %i: %s", i+1 ,location);
				}
			}
			else 
			{
				ROS_WARN("can't get location");
			}
			
			geometry_msgs::Pose pose; // save a pose for moving arm part
			// Checking a part using the logical camera
			int n = 0;
			for (long unsigned int i = 0; i < logical_cam_image.models.size(); i++ ) {
				osrf_gear::Model m = logical_cam_image.models[i];
				if (strcmp(m.type.c_str(), object_type) == 0){
					ROS_INFO("Object: %s | Location: %f, %f, %f | Pose: %f, %f, %f, %f", object_type, 
						m.pose.position.x, m.pose.position.y, m.pose.position.z,
						m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w);
					n++;
					if (n = 3) {
						pose = m.pose; //chose the third part that is observed from the camera
					}
				}
			}
			
			//Transform to find the location of the part in the world
			// Retrieve the transformation
			geometry_msgs::TransformStamped tfStamped;
			try {
				// tf2_ros::Buffer.lookupTransform(“to_frame”, “from_frame”, “how_recent”, “how_long_to_wait”);
				tfStamped = tfBuffer.lookupTransform(/*move_group.getPlanningFrame().c_str()*/"world",
					"logical_camera_frame", ros::Time(0.0), ros::Duration(1.0));
				ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
			} 
			catch (tf2::TransformException &ex) {
				ROS_ERROR("%s", ex.what());
			}
			
			
			// Create variables
			geometry_msgs::PoseStamped part_pose, goal_pose;
			// Copy pose from the logical camera.
			part_pose.pose = pose;

			tf2::doTransform(part_pose, goal_pose, tfStamped);

			// Add height to the goal pose.
			goal_pose.pose.position.z += 0.10; // 10 cm above the part
			// Tell the end effector to rotate 90 degrees around the y-axis (in quaternions… more on quaternions later in the semester).
			goal_pose.pose.orientation.w = 0.707;
			goal_pose.pose.orientation.x = 0.0;
			goal_pose.pose.orientation.y = 0.707;
			goal_pose.pose.orientation.z = 0.0;

			// Set the desired pose for the arm in the arm controller.
			move_group.setPoseTarget(goal_pose);
			// Instantiate and create a plan.
			moveit::planning_interface::MoveGroupInterface::Plan the_plan;
			// Create a plan based on the settings (all default settings now) in the_plan.
			move_group.plan(the_plan);
			// Planning does not always succeed. Check the output.

			// In the event that the plan was created, execute it.
			move_group.execute(the_plan);

			}
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
