 #include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>

int x_position_pickup = 0;
int y_position_pickup = 0;

int x_position_dropoff = 0;
int y_position_dropoff = 0;

int marker_condition = 0;


 int main( int argc, char** argv )
 {
   ros::init(argc, argv, "add_markers");
   ros::NodeHandle n;

   ros::Rate r(1);
   ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

   // Set our initial shape type to be a cube
   uint32_t shape = visualization_msgs::Marker::CUBE;

     visualization_msgs::Marker marker;
     // Set the frame ID and timestamp.  See the TF tutorials for information on these.
     marker.header.frame_id = "/map";
     marker.header.stamp = ros::Time::now();
 
     // Set the namespace and id for this marker.  This serves to create a unique ID
     // Any marker sent with the same namespace and id will overwrite the old one
     marker.ns = "basic_shapes";
     marker.id = 0;
 
     // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
     marker.type = shape;
 
     // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
     //marker.action = visualization_msgs::Marker::ADD;
 
     // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
     //marker.pose.position.x = 0;
     //marker.pose.position.y = 0;
     marker.pose.position.z = 0.17;
     marker.pose.orientation.x = 0.0;
     marker.pose.orientation.y = 0.0;
     marker.pose.orientation.z = 0.0;
     marker.pose.orientation.w = 1.0;
 
     // Set the scale of the marker -- 1x1x1 here means 1m on a side
     marker.scale.x = 0.3;
     marker.scale.y = 0.3;
     marker.scale.z = 0.3;
 
     // Set the color -- be sure to set alpha to something non-zero!
     marker.color.r = 0.0f;
     marker.color.g = 0.0f;
     marker.color.b = 1.0f;
     marker.color.a = 1.0;
 
     marker.lifetime = ros::Duration();
 
     // Publish the marker
     while (marker_pub.getNumSubscribers() < 1)
     {
       if (!ros::ok())
       {
         return 0;
       }
       ROS_WARN_ONCE("Please create a subscriber to the marker");
       sleep(1);
     }

    n.getParam("marker",marker_condition);

     
     shape = visualization_msgs::Marker::CUBE;
     while(marker_condition==0) //place the marker at pickup location
     {
        n.getParam("pickup_x",x_position_pickup);
        n.getParam("pickup_y",y_position_pickup);
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x_position_pickup;
        marker.pose.position.y = y_position_pickup;
        marker_pub.publish(marker);
        ROS_INFO("picking up");
        n.getParam("marker",marker_condition);
     }
     
     while(marker_condition==1)  //remove the marker from pickup location (to simulate pickup)
     {
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
      ROS_INFO("picked up");
      n.getParam("marker",marker_condition);
     }
    while(marker_condition==2)   // place the marker at dropoff location (to simulate dropoff)
      {
      n.getParam("dropoff_x",x_position_dropoff);
      n.getParam("dropoff_y",y_position_dropoff);
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = x_position_dropoff;
      marker.pose.position.y = y_position_dropoff;
      marker_pub.publish(marker);
      ROS_INFO("dropping off");
      n.getParam("marker",marker_condition);
    }
  
      r.sleep();
   
 }