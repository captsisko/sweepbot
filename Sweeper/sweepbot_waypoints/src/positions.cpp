#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"

#include <sstream>
// #include"ros/serialization.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoints");
	// ros::init(argc, argv, "talker");

	ros::NodeHandle nh;
	ros::Publisher waypoint_pub = nh.advertise<visualization_msgs::Marker>("waypoint_marker", 0);
	// ros::Publisher waypoint_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
	// ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);

	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	geometry_msgs::Point p_1;
	geometry_msgs::Point p_2;
	geometry_msgs::Point p_3;

	p_1.x = 0.0;
	p_1.y = 0.0;
	p_1.z = 0.0;

	p_2.x = 1.0;
	p_2.y = 1.0;
	p_2.z = 1.0;

	p_3.x = 2.0;
	p_3.y = 2.0;
	p_3.z = 2.0;

	std::vector<geometry_msgs::Point> points;
	points.push_back(p_1);
	points.push_back(p_2);
	points.push_back(p_3);

	for( int i1 = 0; i1 < points.size(); ++i1 )
	{
		std_msgs::ColorRGBA c;
		if( i1 == 0 )
			c.r = 1.0;
		else if( i1 == 1 )
			c.g = 1.0;
		else
			c.b = 1.0;
		c.a = 1.0;

		marker.points.push_back(points[i1]);
		marker.colors.push_back(c);
	}

	// ROS_INFO("%s", 'test');

	waypoint_pub.publish(marker);
	ros::spin();
}