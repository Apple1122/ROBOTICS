#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>
using namespace visualization_msgs;

Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

double rand( double min, double max )
{
  double t = (double)rand() / (double)RAND_MAX;
  return min + t*(max-min);
}

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_marker");

  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("simple_marker");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "my_marker";
  int_marker.description = "Simple 6-DOF Control";

 // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.45;
  box_marker.scale.y = 0.45;
  box_marker.scale.z = 0.45;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;



  // my code
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back(box_control);

  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode =  visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode =  visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);


  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode =  visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode =  visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode =  visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode =  visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);


  server.insert(int_marker, &processFeedback);




  tf::Vector3 position;

  InteractiveMarker int_marker2;
  int_marker2.header.frame_id = "moving_frame";

  position = tf::Vector3(-3,-3, 0);

  tf::pointTFToMsg(position, int_marker2.pose.position);
  int_marker2.scale = 1;

   

  int_marker2.name = "moving";
  int_marker2.description = "Marker Attached to a\nMoving Frame";

  InteractiveMarkerControl control2;

  tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation);
  control2.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker2.controls.push_back(control2);

  control2.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control2.always_visible = true;
  control2.markers.push_back( makeBox(int_marker2) );
  int_marker2.controls.push_back(control2);

  server.insert(int_marker2, &processFeedback);





  InteractiveMarker int_marker3;
  int_marker3.header.frame_id = "base_link";
  position = tf::Vector3( 3, 0, 0);
  tf::pointTFToMsg(position, int_marker3.pose.position);
  int_marker3.scale = 1;

  int_marker3.name = "quadrocopter";
  int_marker3.description = "Quadrocopter";

  makeBoxControl(int_marker3);

  InteractiveMarkerControl control3;

  tf::Quaternion orien2(0.0, 1.0, 0.0, 1.0);
  orien2.normalize();
  tf::quaternionTFToMsg(orien2, control3.orientation);
  control3.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  int_marker3.controls.push_back(control3);
  control3.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker3.controls.push_back(control3);



  server.insert(int_marker3, &processFeedback);




  InteractiveMarker int_marker4;
  int_marker4.header.frame_id = "base_link";
  position = tf::Vector3( 3, 3, 3);
  tf::pointTFToMsg(position, int_marker4.pose.position);
  int_marker4.scale = 1;

  int_marker4.name = "6dof_random_axes";
  int_marker4.description = "6-DOF\n(Arbitrary Axes)";

  makeBoxControl(int_marker4);

  InteractiveMarkerControl control4;

  for ( int i=0; i<3; i++ )
  {
    tf::Quaternion orien4(rand(-1,1), rand(-1,1), rand(-1,1), rand(-1,1));
    orien4.normalize();
    tf::quaternionTFToMsg(orien4, control4.orientation);
    control4.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker4.controls.push_back(control4);
    control4.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker4.controls.push_back(control4);
  }


  server.insert(int_marker4, &processFeedback);


  // 'commit' changes and send to all clients
  server.applyChanges();

  // start the ROS main loop
  ros::spin();
}
