#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int8.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "add_markers_time");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;
    
    while (ros::ok()) {
        visualization_msgs::Marker marker;
        
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.5f;
        marker.color.g = 1.0f;
        marker.color.b = 0.5f;
        marker.color.a = 1.0;
        // Set the namespace and id for this marker. This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "cube";
        marker.id = 0;
        // Set the marker type
        marker.type = shape;

        for (int i=0; i<2; i++) {
            // Set the frame ID and timestamp
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            // Set the marker action
            marker.action = visualization_msgs::Marker::ADD;
            
            // Set the pose of the marker the goal pose
            if ( i == 0) {
                marker.pose.position.x = 1.0;
                marker.pose.position.y = 0.0;
                marker.pose.position.z = 0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
            } else {
                marker.pose.position.x = -1.0;
                marker.pose.position.y = -1.0;
                marker.pose.position.z = 0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
            }
        
            // Publish the marker
            while (marker_pub.getNumSubscribers() < 1) {
                if (!ros::ok()) {
                    return 0;
                }
                ROS_WARN_ONCE("Please create a subscriber to the marker");
                ros::Duration(0.5,0).sleep();
            }
            
            if (i == 0) {
                marker_pub.publish(marker);
                ROS_INFO("Marker at pickup point");
                ROS_INFO("Waiting for 5 seconds...");
                ros::Duration(5).sleep();
                ROS_INFO("Robot reached the pickup point");
                ros::Duration(0.5,0).sleep();
                marker.action = visualization_msgs::Marker::DELETE;
                marker_pub.publish(marker);
                ROS_INFO("Marker picked up by robot");
                ros::Duration(5).sleep();
            } else {
                ROS_INFO("Robot reached the dropoff point");
                ros::Duration(0.5).sleep();
                marker.action = visualization_msgs::Marker::ADD;
                marker_pub.publish(marker);
                ros::Duration(5).sleep();
                ROS_INFO("Marker dropped off by robot");
            }

        r.sleep();
        }
    }
    return 0;
}
