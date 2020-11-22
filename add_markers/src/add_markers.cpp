#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>

uint8_t goal_status_number = 0;

void goalStatusCallBack(const std_msgs::UInt8::ConstPtr& msg)
{
	goal_status_number = msg->data;
	return;
} 


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber goal_sub = n.subscribe("/goal_status",1,goalStatusCallBack);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  bool complete = false;
  while(ros::ok())
  {
  	ros::spinOnce();
  	visualization_msgs::Marker marker;
  	marker.header.frame_id = "/map";
  	marker.header.stamp = ros::Time::now();
  	marker.ns = "basic_shapes";
  	marker.id = 0;
  	marker.type = shape;
  	marker.scale.x = 0.4;
  	marker.scale.y = 0.4;
  	marker.scale.z = 0.4;
  	marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    
    switch(goal_status_number)
    {
    	case 0:
    		{
    			marker.action = visualization_msgs::Marker::ADD;
    			marker.pose.position.x = 7.0;
    			marker.pose.position.y = -2.0;
    			marker.pose.position.z = 0.0;
    			marker.pose.orientation.x = 0.0;
    			marker.pose.orientation.y = 0.0;
    			marker.pose.orientation.z = 0.0;
    			marker.pose.orientation.w = 2.0;
    			break;
    		}
    	case 1:
    		{
    			marker.action = visualization_msgs::Marker::DELETE;
    			break;
    		}
    	case 2:
    		{
    			marker.action = visualization_msgs::Marker::ADD;
    			marker.pose.position.x = -3.0;
    			marker.pose.position.y = 11.0;
    			marker.pose.position.z = 0.0;
    			marker.pose.orientation.x = 0.0;
    			marker.pose.orientation.y = 0.0;
    			marker.pose.orientation.z = 0.0;
    			marker.pose.orientation.w = 2.0;
    			complete = true;
    			break;
    		}
    }
    
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    
    marker_pub.publish(marker);
    
    if (complete) {
      ROS_INFO("Object Dropped-Off");
      sleep(7);
      return 0;
      }

    r.sleep();
  }
  
	return 0;
 
}
